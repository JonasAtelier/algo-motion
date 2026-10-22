# Runs each library's own Makefile in turn, stopping at the first failure.
# Every library still builds and tests on its own; this only saves the cd.
LIBS   := $(patsubst %/Makefile,%,$(wildcard */Makefile))
NOMAKE := $(filter-out $(LIBS),$(patsubst %/README.md,%,\
            $(wildcard */README.md)))

check clean:
	@for d in $(NOMAKE); do echo "SKIP: $$d (no Makefile)"; done
	@for d in $(LIBS); do $(MAKE) -C $$d $@ || exit 1; done

.PHONY: check clean
