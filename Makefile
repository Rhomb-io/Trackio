all:
	platformio -f run

upload:
	platformio -f run --target upload

clean:
	platformio -f run --target clean

program:
	platformio -f run --target program

uploadfs:
	platformio -f run --target uploadfs

update:
	platformio -f update

check-all:
	cppcheck ./ --enable=all -I lib/Trackio

check:
	cppcheck ./lib/Trackio/Trackio.cpp --enable=all

build-docs:
	mkdocs build

server-docs:
	mkdocs serve
