DESTDIR=$
PREFIX=/usr/local/

all:
	gcc main.m -o termz

install:
	mkdir -p $(DESTDIR)$(PREFIX)bin/
	cp termz $(DESTDIR)$(PREFIX)bin/

uninstall:
	rm $(DESTDIR)$(PREFIX)bin/termz
