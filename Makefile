HEADERS = MY17_Can_Library.h bms.h can_node.h can_raw.h current_sensor.h dash.h evil_macros.h ids.h mc.h vcu.h can_validator/fsae_can_spec.h

default: main

obj/evil_macros.o: evil_macros.c $(HEADERS)
	gcc -c evil_macros.c -o obj/evil_macros.o -DCAN_ARCHITECTURE_TEST

obj/MY17_Can_Library.o: MY17_Can_Library.c $(HEADERS)
	gcc -c MY17_Can_Library.c -o obj/MY17_Can_Library.o -DCAN_ARCHITECTURE_TEST

obj/main.o: src_test/main.c $(HEADERS)
	gcc -c src_test/main.c -o obj/main.o -DCAN_ARCHITECTURE_TEST

main: obj/main.o obj/MY17_Can_Library.o obj/evil_macros.o
	gcc -o obj/main obj/main.o obj/MY17_Can_Library.o obj/evil_macros.o -DCAN_ARCHITECTURE_TEST

clean:
	-rm -f obj/*.o
	-rm -f obj/main
