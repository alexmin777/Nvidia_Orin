cmd_scripts/kconfig/lxdialog/yesno.o := gcc -Wp,-MMD,scripts/kconfig/lxdialog/.yesno.o.d -Wall -Wmissing-prototypes -Wstrict-prototypes -O2 -fomit-frame-pointer -std=gnu89      -D_GNU_SOURCE -D_DEFAULT_SOURCE -c -o scripts/kconfig/lxdialog/yesno.o scripts/kconfig/lxdialog/yesno.c

source_scripts/kconfig/lxdialog/yesno.o := scripts/kconfig/lxdialog/yesno.c

deps_scripts/kconfig/lxdialog/yesno.o := \
  scripts/kconfig/lxdialog/dialog.h \

scripts/kconfig/lxdialog/yesno.o: $(deps_scripts/kconfig/lxdialog/yesno.o)

$(deps_scripts/kconfig/lxdialog/yesno.o):
