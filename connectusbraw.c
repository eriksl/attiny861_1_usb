#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <libusb-1.0/libusb.h>

static void debug_packet(const char *id, int length, const uint8_t *data)
{
	int ix;

	printf("%s[%d]> ", id, length);

	for(ix = 0; ix < length; ix++)
		printf("%02x ", data[ix]);

	printf("\n");
}

static void send_packet(libusb_device_handle *handle, size_t length, uint8_t *packet)
{
	ssize_t rv;

	rv = libusb_control_transfer(handle,
			LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT,
			0, 0, 0, 
			packet, length, 0);

	if(rv < 0)
		fprintf(stderr, "send_packet: %s\n", libusb_error_name(rv));
}

static size_t receive_packet(libusb_device_handle *handle, size_t length, uint8_t *packet)
{
	ssize_t rv;

	rv = libusb_control_transfer(handle,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT,
			0, 0, 0, 
			packet, length, 1000);

	if(rv < 0)
	{
		fprintf(stderr, "libusb_control_transfer(receive): %s\n", libusb_error_name(rv));
		rv = 0;
	}

	return(rv);
}

static void communicate(libusb_device_handle *handle)
{
	uint8_t in_packet[128];
	uint8_t out_packet[128];
	ssize_t	length;

	out_packet[0] = 0;

	send_packet(handle, 1, out_packet);
	length = receive_packet(handle, sizeof(in_packet), in_packet);

	debug_packet("identify", length, in_packet);
}

int main(int argc, char **argv)
{
	ssize_t							rv, ix;
	libusb_device					**device_list;
	libusb_device					*device;
	struct libusb_device_descriptor	descriptor;
	libusb_device_handle			*handle;
	uint8_t							manufacturer[32];
	uint8_t							product[32];

	if((rv = libusb_init(0)) != 0)
	{
		fprintf(stderr, "libusb_init: %s\n", libusb_error_name(rv));
		return(-1);
	}

	libusb_set_debug(0, 3);

	if((rv = libusb_get_device_list(0, &device_list)) < 0)
	{
		fprintf(stderr, "libusb_get_device_list: %s\n", libusb_error_name(rv));
		return(-1);
	}

	for(ix = 0; (device = device_list[ix]); ix++)
	{
		if(libusb_get_device_descriptor(device, &descriptor) < 0)
			continue;

		if(libusb_open(device, &handle) < 0)
			continue;

		rv = libusb_get_string_descriptor_ascii(handle, descriptor.iManufacturer, manufacturer, sizeof(manufacturer));

		if(rv < 0)
			manufacturer[0] = 0;

		rv = libusb_get_string_descriptor_ascii(handle, descriptor.iProduct, product, sizeof(product));

		if(rv < 0)
			product[0] = 0;

		//printf("device: %04x:%04x, version: %04x, manufacturer: %s, product: %s\n",
				//descriptor.idVendor, descriptor.idProduct, descriptor.bcdDevice,
				//manufacturer, product);

		libusb_close(handle);

		if(!strcmp((char *)manufacturer, "slagter.name") && !strcmp((char *)product, "vusb-test"))
		{
			libusb_ref_device(device);
			break;
		}
	}

	libusb_free_device_list(device_list, 1);

	if(device)
	{
		if((rv = libusb_get_device_descriptor(device, &descriptor)))
		{
			fprintf(stderr, "libusb_get_device_descriptor: %s\n", libusb_error_name(rv));
			return(-1);
		}

		printf("found at %04x:%04x = %04x\n", 
			descriptor.idVendor, descriptor.idProduct, descriptor.bcdDevice);

		if((rv = libusb_open(device, &handle) < 0))
		{
			fprintf(stderr, "libusb_open: %s\n", libusb_error_name(rv));
			return(-1);
		}

		libusb_detach_kernel_driver(handle, 0);

		if((rv = libusb_set_configuration(handle, 1)) < 0)
		{
			fprintf(stderr, "libusb_set_configuration: %s\n", libusb_error_name(rv));
			return(-1);
		}

		if((rv = libusb_claim_interface(handle, 0)) < 0)
		{
			fprintf(stderr, "libusb_claim_interface: %s\n", libusb_error_name(rv));
			return(-1);
		}

		communicate(handle);

		if((rv = libusb_release_interface(handle, 0)) < 0)
		{
			fprintf(stderr, "libusb_release_interface: %s\n", libusb_error_name(rv));
			return(-1);
		}

		libusb_attach_kernel_driver(handle, 0);
		libusb_close(handle);
		libusb_unref_device(device);
	}

	libusb_exit(0);

	return(0);
}
