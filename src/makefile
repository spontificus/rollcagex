#if ode came with drawstuff:
#drawstuff=-ldrawstuff
#if ode didn't come with drawstuff (use the shipped drawstuff):
drawstuff=../drawstuff/src/libdrawstuff.a
#uncomment one of the above

cc=gcc
cflags=-O2
objs=main.c main.h dynamics.c graphics.c
libs=-lGL -lGLU -lode $(drawstuff)

rcx: $(objs)
	$(cc) main.c $(cflags) $(libs) -o ../$@

#just to simplify testing, cus I'm lazy
test: rcx
	(cd ..&&./rcx)
#end of lazy

.PHONY: clean
clean:
	rm -f rcx rcx.o
