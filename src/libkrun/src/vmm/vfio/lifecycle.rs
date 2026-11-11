// Copyright 2026 The libkrun Authors.
// SPDX-License-Identifier: Apache-2.0

use super::release_all;
use polly::event_manager::{EventManager, Subscriber};
use std::io;
use std::os::fd::AsRawFd;
use std::sync::{
    Arc,
    atomic::{AtomicI32, Ordering},
};
use utils::epoll::{EpollEvent, EventSet};
use utils::eventfd::EventFd;
use vfio_bindings::bindings::vfio::*;
use vfio_ioctls::VfioIrq;

struct Notification {
    index: u32,
    event: EventFd,
}

fn notification_name(index: u32) -> &'static str {
    match index {
        VFIO_PCI_ERR_IRQ_INDEX => "ERR (fatal device error)",
        VFIO_PCI_REQ_IRQ_INDEX => "REQ (host release request)",
        _ => unreachable!(),
    }
}

pub(super) struct Lifecycle {
    notifications: Vec<Notification>,
    reason: Option<String>,
    exit_evt: EventFd,
    exit_code: Arc<AtomicI32>,
}

impl Lifecycle {
    pub fn new(exit_evt: EventFd, exit_code: Arc<AtomicI32>) -> Self {
        Self {
            notifications: Vec::new(),
            reason: None,
            exit_evt,
            exit_code,
        }
    }

    pub fn register(
        &mut self,
        index: u32,
        info: Option<&VfioIrq>,
        enable: impl FnOnce(&EventFd) -> io::Result<()>,
    ) -> io::Result<()> {
        let Some(info) = info.filter(|info| info.count != 0) else {
            return Ok(());
        };
        let name = notification_name(index);
        if info.count != 1 || info.flags & VFIO_IRQ_INFO_EVENTFD == 0 {
            return Err(io::Error::other(format!(
                "VFIO {name} notification does not support one eventfd"
            )));
        }
        let event = EventFd::new(libc::EFD_NONBLOCK | libc::EFD_CLOEXEC)
            .map_err(|e| io::Error::other(format!("VFIO {name} eventfd: {e}")))?;
        enable(&event).map_err(|e| {
            io::Error::other(format!("VFIO {name} notification registration failed: {e}"))
        })?;
        self.notifications.push(Notification { index, event });
        Ok(())
    }

    pub fn disable(&mut self, mut disable: impl FnMut(u32) -> io::Result<()>) -> io::Result<()> {
        release_all(&mut self.notifications, |notification| {
            disable(notification.index).map_err(|e| {
                io::Error::other(format!(
                    "VFIO {} notification disable failed: {e}",
                    notification_name(notification.index)
                ))
            })
        })
    }

    pub fn failed(&self) -> bool {
        self.reason.is_some()
    }

    pub fn fail(&mut self, reason: String) {
        if self.failed() {
            return;
        }
        log::error!("VFIO device failure: {reason}");
        self.reason = Some(reason);
        self.exit_code.store(1, Ordering::SeqCst);
        if let Err(error) = self.exit_evt.write(1) {
            log::error!("VFIO exit notification: {error}");
        }
    }
}

impl Subscriber for Lifecycle {
    fn process(&mut self, event: &EpollEvent, _: &mut EventManager) {
        let Some(notification) = self
            .notifications
            .iter()
            .find(|n| n.event.as_raw_fd() == event.fd())
        else {
            return;
        };
        let name = notification_name(notification.index);
        if !event.event_set().contains(EventSet::IN) {
            self.fail(format!("{name} notification poll error"));
            return;
        }
        match notification.event.read() {
            Ok(_) => self.fail(name.into()),
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => (),
            Err(error) => self.fail(format!("{name} notification read: {error}")),
        }
    }

    fn interest_list(&self) -> Vec<EpollEvent> {
        self.notifications
            .iter()
            .map(|notification| {
                EpollEvent::new(EventSet::IN, notification.event.as_raw_fd() as u64)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    fn lifecycle() -> Lifecycle {
        Lifecycle::new(
            EventFd::new(libc::EFD_NONBLOCK).unwrap(),
            Arc::new(AtomicI32::new(i32::MAX)),
        )
    }
    fn info(index: u32) -> VfioIrq {
        VfioIrq {
            index,
            count: 1,
            flags: VFIO_IRQ_INFO_EVENTFD,
        }
    }

    #[test]
    fn vfio_notifications_registration_and_partial_cleanup() {
        let mut lifecycle = lifecycle();
        for index in [VFIO_PCI_ERR_IRQ_INDEX, VFIO_PCI_REQ_IRQ_INDEX] {
            for info in [
                None,
                Some(VfioIrq {
                    count: 0,
                    ..info(index)
                }),
            ] {
                lifecycle
                    .register(index, info.as_ref(), |_| panic!("unsupported"))
                    .unwrap();
            }
            assert!(
                lifecycle
                    .register(
                        index,
                        Some(&VfioIrq {
                            flags: 0,
                            ..info(index)
                        }),
                        |_| panic!("no eventfd support"),
                    )
                    .is_err()
            );
            let result = lifecycle.register(index, Some(&info(index)), |_| {
                if index == VFIO_PCI_REQ_IRQ_INDEX {
                    Err(io::Error::other("injected registration error"))
                } else {
                    Ok(())
                }
            });
            if index == VFIO_PCI_REQ_IRQ_INDEX {
                let error = result.unwrap_err().to_string();
                assert!(error.contains(notification_name(index)));
                assert!(error.contains("registration failed"));
            } else {
                result.unwrap();
            }
        }
        assert_eq!(lifecycle.notifications.len(), 1);
        assert_eq!(lifecycle.notifications[0].index, VFIO_PCI_ERR_IRQ_INDEX);
        lifecycle
            .register(
                VFIO_PCI_REQ_IRQ_INDEX,
                Some(&info(VFIO_PCI_REQ_IRQ_INDEX)),
                |_| Ok(()),
            )
            .unwrap();
        let mut attempts = vec![];
        assert!(
            lifecycle
                .disable(|index| {
                    attempts.push(index);
                    if index == VFIO_PCI_ERR_IRQ_INDEX {
                        Err(io::Error::other("disable ERR"))
                    } else {
                        Ok(())
                    }
                })
                .is_err()
        );
        assert_eq!(attempts, [VFIO_PCI_ERR_IRQ_INDEX, VFIO_PCI_REQ_IRQ_INDEX]);
        assert_eq!(lifecycle.notifications.len(), 1);
        assert_eq!(lifecycle.notifications[0].index, VFIO_PCI_ERR_IRQ_INDEX);
        lifecycle.disable(|_| Ok(())).unwrap();
    }

    #[test]
    fn vfio_err_and_req_event_loop_delivery_stop_once_and_record_reason() {
        for (first, index) in [VFIO_PCI_ERR_IRQ_INDEX, VFIO_PCI_REQ_IRQ_INDEX]
            .into_iter()
            .enumerate()
        {
            let mut lifecycle = lifecycle();
            for index in [VFIO_PCI_ERR_IRQ_INDEX, VFIO_PCI_REQ_IRQ_INDEX] {
                lifecycle
                    .register(index, Some(&info(index)), |_| Ok(()))
                    .unwrap();
            }
            let triggers: Vec<_> = lifecycle
                .notifications
                .iter()
                .map(|n| n.event.try_clone().unwrap())
                .collect();
            let lifecycle = Arc::new(Mutex::new(lifecycle));
            let mut manager = EventManager::new().unwrap();
            manager.add_subscriber(lifecycle.clone()).unwrap();
            for (iteration, trigger) in [first, 1 - first, first].into_iter().enumerate() {
                triggers[trigger].write(2).unwrap();
                assert_eq!(manager.run_with_timeout(0).unwrap(), 1);
                let lifecycle = lifecycle.lock().unwrap();
                assert!(lifecycle.failed());
                assert_eq!(lifecycle.reason.as_deref(), Some(notification_name(index)));
                assert_eq!(lifecycle.exit_code.load(Ordering::SeqCst), 1);
                if iteration == 0 {
                    assert_eq!(lifecycle.exit_evt.read().unwrap(), 1);
                } else {
                    assert_eq!(
                        lifecycle.exit_evt.read().unwrap_err().kind(),
                        io::ErrorKind::WouldBlock
                    );
                }
            }
            assert_eq!(manager.run_with_timeout(0).unwrap(), 0);
        }
    }
}
