#if ode came with drawstuff:
#drawstuff=-ldrawstuff
#if ode didn't come with drawstuff (use the shipped drawstuff):
drawstuff=drawstuff/src/libdrawstuff.a
#uncomment one of the above

cc=gcc
cflags=-O2
objs=main.c
libs=-lGL -lGLU -lode $(drawstuff)

rcx: $(objs)
	$(cc) $(cflags) $(objs) $(libs) -o $@

.PHONY: clean
clean:
	rm -f rcx rcx.o
