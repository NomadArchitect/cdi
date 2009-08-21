SRCS = $(shell find -name '*.c')
OBJS = $(addsuffix .o,$(basename $(SRCS)))

CC = gcc
CFLAGS = -m32 -Wall -g -I include -Werror -D_GNU_SOURCE

run_command = $(if $(V),$(2),@echo $(1);$(2))

all: $(OBJS)

%.o: %.c
	$(call run_command,"CC $^",$(CC) $(CFLAGS) $(addprefix -I,$(addprefix $(dir $^),. include)) -c -o $@ $^)

clean:
	rm $(OBJS)

.PHONY: clean
