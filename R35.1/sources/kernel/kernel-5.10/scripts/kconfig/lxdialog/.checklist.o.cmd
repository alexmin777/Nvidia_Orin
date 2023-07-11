cmd_scripts/kconfig/lxdialog/checklist.o := gcc -Wp,-MMD,scripts/kconfig/lxdialog/.checklist.o.d -Wall -Wmissing-prototypes -Wstrict-prototypes -O2 -fomit-frame-pointer -std=gnu89      -D_GNU_SOURCE -D_DEFAULT_SOURCE -c -o scripts/kconfig/lxdialog/checklist.o scripts/kconfig/lxdialog/checklist.c

source_scripts/kconfig/lxdialog/checklist.o := scripts/kconfig/lxdialog/checklist.c

deps_scripts/kconfig/lxdialog/checklist.o := \
  scripts/kconfig/lxdialog/dialog.h \

scripts/kconfig/lxdialog/checklist.o: $(deps_scripts/kconfig/lxdialog/checklist.o)

$(deps_scripts/kconfig/lxdialog/checklist.o):
