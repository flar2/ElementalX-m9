#ifndef __USB_CONTROLLER_H__
#define __USB_CONTROLLER_H__

struct usb_controller {
	int (*notify_attached_source)(struct usb_controller *uc, int value);
};

extern int usb_controller_register(struct device* parent, struct usb_controller *uc);

#endif
