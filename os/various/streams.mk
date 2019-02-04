# RT Shell files.
STREAMSSRC = $(CHIBIOS)/os/various/chprintf.c \
             $(CHIBIOS)/os/various/memstreams.c 

STREAMSINC = $(CHIBIOS)/os/various

# Shared variables
ALLCSRC += $(STREAMSSRC)
ALLINC  += $(STREAMSINC)
