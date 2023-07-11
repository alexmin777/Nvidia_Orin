cmd_scripts/kconfig/lxdialog/util.o := gcc -Wp,-MMD,scripts/kconfig/lxdialog/.util.o.d -Wall -Wmissing-prototypes -Wstrict-prototypes -O2 -fomit-frame-pointer -std=gnu89      -D_GNU_SOURCE -D_DEFAULT_SOURCE -c -o scripts/kconfig/lxdialog/util.o scripts/kconfig/lxdialog/util.c

source_scripts/kconfig/lxdialog/util.o := scripts/kconfig/lxdialog/util.c

deps_scripts/kconfig/lxdialog/util.o := \
  scripts/kconfig/lxdialog/dialog.h \

scripts/kconfig/lxdialog/util.o: $(deps_scripts/kconfig/lxdialog/util.o)

$(deps_scripts/kconfig/lxdialog/util.o):
