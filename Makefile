HEADERS = MY17_Can_Library.h bms.h can_node.h can_raw.h current_sensor.h dash.h evil_macros.h ids.h mc.h vcu.h can_validator/fsae_can_spec.h

default: MY17_Can_Library

MY17_Can_Library.o: MY17_Can_Library.c $(HEADERS)
	gcc -c MY17_Can_Library.c -o obj/MY17_Can_Library.o -DCAN_ARCHITECTURE_TEST

MY17_Can_Library: MY17_Can_Library.o
	gcc obj/MY17_Can_Library.o -o obj/MY17_Can_Library -DCAN_ARCHITECTURE_TEST

clean:
	-rm -f obj/MY17_Can_Library.o
	-rm -f obj/MY17_Can_Library
