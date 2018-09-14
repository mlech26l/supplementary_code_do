
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin

TARGET   = endeffector

TARGET_LIB = endeffector.o

OPT = -O3
CXX       = clang++
# compiling flags here
CFLAGS   =  -std=c++11 $(OPT) -Wall -I.


LINKER   = clang++ -o
# linking flags here
LFLAGS   = -Wall $(OPT) -I. -lm

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
LIB_OBJECTS := $(filter-out $(OBJDIR)/endeffector_test.o,$(OBJECTS))

rm       = rm -f


all: $(BINDIR)/$(TARGET)

lib: $(BINDIR)/$(TARGET_LIB)

$(BINDIR)/$(TARGET_LIB): $(LIB_OBJECTS)
	ar cr $@ $(LIB_OBJECTS)
	@echo "Built lib successfully!"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	$(CXX) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

$(BINDIR)/$(TARGET): $(OBJECTS)
	$(LINKER) $@ $(LFLAGS) $(OBJECTS)
	@echo "Linking complete!"

clean:
	rm -f bin/$(TARGET_LIB)
	rm -f bin/$(TARGET)
	rm -f obj/*.o
