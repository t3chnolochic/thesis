/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#define LBLUE GPIOE, GPIO8
#define LRED GPIOE, GPIO9
#define LORANGE GPIOE, GPIO10
#define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
#define LRED2 GPIOE, GPIO13
#define LORANGE2 GPIOE, GPIO14
#define LGREEN2 GPIOE, GPIO15

#define LD4 GPIOE, GPIO8
#define LD3 GPIOE, GPIO9
#define LD5 GPIOE, GPIO10
#define LD7 GPIOE, GPIO11
#define LD9 GPIOE, GPIO12
#define LD10 GPIOE, GPIO13
#define LD8 GPIOE, GPIO14
#define LD6 GPIOE, GPIO15


static usbd_device * usb_device;
static volatile uint8_t usb_did_first_read = 0;
static volatile uint8_t usb_ready_to_send = 0;
static volatile uint8_t sig = 0;


void adc1_2_isr(void) {
    
    if ( adc_eoc(ADC1) != 0 ) {
        sig = adc_read_regular(ADC1);
        gpio_port_write(GPIOE, (sig << 8) | 0x0001);
        DAC_DHR8R1 = DAC_DHR8R1_DACC1DHR_MSK & sig;
        //DAC_DHR12R1 = DAC_DHR12R1_DACC1DHR_MSK & 0x0FFF;
        ADC1_CR |= ADC_CR_ADSTART;
        
    }
    
}


static void adc_setup(void)
{
	//ADC
	rcc_periph_clock_enable(RCC_ADC12);
	rcc_periph_clock_enable(RCC_GPIOA);
	//ADC
	//gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0); //pa0 //dead
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1); //pa1
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2); //pa2
    gpio_mode_setup(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4); //f4
    
    if ((ADC1_CR & ADC_CR_ADEN) != 0 ){ //basically is ADC read yet. wait until ADDIS is done
        ADC1_CR |= ADC_CR_ADDIS;
        while (ADC1_CR & ADC_CR_ADDIS){}
    }
    
    ADC1_CFGR = ADC_CFGR_EXTEN_DISABLED	| ADC_CFGR_RES_8_BIT ; //ADC_CFGR_CONT |ADC_CFGR_DMAEN | ADC_CFGR_EXTEN_RISING_EDGE |ADC_CFGR_EXTSEL_EVENT_4 |
    
    ADC1_CFGR &= ~ADC_CFGR_ALIGN;
    
	
    ADC_CCR = ADC_CCR_CKMODE_DIV1;
    ADC1_IER = ADC_IER_EOCIE; //only on if you want to see one conversion at a time

    ADC1_CR = ADC_CR_ADVREGEN_INTERMEDIATE;
    ADC1_CR = ADC_CR_ADVREGEN_ENABLE;

	ADC1_SMPR1 = (ADC_SMPR1_SMP_19DOT5CYC) << 9;
    //ADC_SMPR1_SMP_1DOT5CYC = 4.8MSPS
    //ADC_SMPR1_SMP_2DOT5CYC = 4.36MSPS
    //ADC_SMPR1_SMP_4DOT5CYC = 3.7MSPS
    //ADC_SMPR1_SMP_7DOT5CYC = 3.0MSPS, gets a bit unstable if higher.
    //ADC_SMPR1_SMP_19DOT5CYC = 1.7MSPS
    //ADC_SMPR1_SMP_61DOT5CYC = 685.7kSPS
    //ADC_SMPR1_SMP_181DOT5CYC = 252.6kSPS
    //ADC_SMPR1_SMP_601DOT5CYC = 78.7kSPS
    
    ADC1_SQR1 = ( ( 3 ) << ADC_SQR1_SQ1_LSB ); //PA2
    
    //calibration
    //ADC1_CR |= ADC_CR_ADCALDIF; single
    ADC1_CR |= ADC_CR_ADCAL; //single ended
    while ((ADC1_CR & ADC_CR_ADCAL) != 0) {}
    
    ADC1_CR |= ADC_CR_ADCALDIF; //diff
    ADC1_CR |= ADC_CR_ADCAL;
    while ((ADC1_CR & ADC_CR_ADCAL) != 0) {}
    
    // power on ADC
    ADC1_CR |= ADC_CR_ADEN;
    
    nvic_enable_irq(NVIC_ADC1_2_IRQ); //only on if you want to see one conversion at a time
    
	/* Wait for ADC starting up. */
    while ((ADC1_ISR & ADC_ISR_ADRDY) == 0){}
    ADC1_ISR = ADC_ISR_ADRDY;
    
}



static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
                    GPIO14 | GPIO15);
    
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO0); //ADC_EOC
}

static void dac_setup(void){
    rcc_periph_clock_enable(RCC_DAC1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4); //DAC pn PA5
    DAC_CR = DAC_CR_EN1;
    //------
    //DAC_DHR8R1 &= DAC_DHR8R1_DACC1DHR_MSK;
    //DAC_DOR1_DACC1DOR_MSK;
    
}

#if 0
static void my_usb_print_int(usbd_device *usbd_dev, int16_t value)
{
    int8_t i = 0;
    uint16_t len = 0;
    char buffer[64];
    
    if (value < 0) {
        buffer[0] = '-';
        len++;
        value = value * -1;
    }

    i = 3;

    while (i >= 0) {
        buffer[len + i] = "0123456789"[value % 10];
        value /= 10;
        i--;
    }

    len += 4;

    buffer[len++] = '\r';
    buffer[len++] = '\n';
    
    usbd_ep_write_packet(usbd_dev,0x82, buffer, len);
}
#endif



static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return 1;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return 0;
		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (len) {
        usb_did_first_read = 1;
		//usbd_ep_write_packet(usbd_dev, 0x82, buf, len);
		//buf[len] = 0;
	}
}

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep) {
	(void)ep;
	(void)usbd_dev;

    usb_ready_to_send = 1;

    #if 0
    uint16_t temp;
    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    temp=adc_read_regular(ADC1);
    gpio_port_write(GPIOE, temp << 4);
    my_usb_print_int(usbd_dev, temp);
    #endif

}



static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	(void)usbd_dev;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_tx_cb);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}


static void usb_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_usb_prescale_1();
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF14, GPIO11| GPIO12);
}



int main(void)
{
    rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_48MHZ]);
    gpio_setup();
	adc_setup();
    usb_setup();
    dac_setup();
	
	usb_device = usbd_init(&stm32f103_usb_driver, &dev, &config, usb_strings,
			3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usb_device, cdcacm_set_config);

    ADC1_CR |= ADC_CR_ADSTART;
    
	while (1){
        gpio_port_write(GPIOE, (sig << 8) | 0x0000);
		//usbd_poll(usbd_device);
        
        /*
        if (adc_eoc(ADC1) == 1){
            ADC1_CR |= ADC_CR_ADSTART;
            gpio_port_write(GPIOE, (sig << 8));
            sig = adc_read_regular(ADC1);
            DAC_DHR8R1 = DAC_DHR8R1_DACC1DHR_MSK & sig;
            //DAC_DHR12R1 = DAC_DHR12R1_DACC1DHR_MSK & 0x0FFF;
        }
        */
        
    }
}
