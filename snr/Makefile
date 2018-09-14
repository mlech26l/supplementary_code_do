
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin

TARGET   = snr

OPT = -O3
CXX       = g++
# compiling flags here
CFLAGS   =  -pthread -std=c++11 $(OPT) -Wall -I. -I../nnpp/src


LINKER   = g++ -o
# linking flags here
LFLAGS   = -pthread -Wall $(OPT) -I. -I. -I../nnpp/src -lm
LIB = ../nnpp/bin/nnpp_lib.o

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

rm       = rm -f


all: $(BINDIR)/$(TARGET)

lib: $(BINDIR)/$(TARGET_LIB)

$(BINDIR)/$(TARGET_LIB): $(LIB_OBJECTS)
	ar cr $@ $(LIB_OBJECTS)
	@echo "Built lib successfully!"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

$(BINDIR)/$(TARGET): $(OBJECTS)
	$(LINKER) $@ $(LFLAGS) $(OBJECTS) $(LIB)
	@echo "Linking complete!"

clean:
	rm -f bin/$(TARGET)
	rm -f obj/*.o
