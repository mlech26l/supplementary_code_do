
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin

TARGET   = arm_top

OPT = -O3
CXX       = clang++
# compiling flags here
CFLAGS   =  -pthread -std=c++11 $(OPT) -Wall -I. -I./../endeffector/src/ -I./../nnpp/src/

LINKER   = clang++ -o
# linking flags here
LFLAGS   = -pthread -Wall $(OPT) -I. -lm

LINK_OBJ = ./../nnpp/bin/nnpp_lib.o  ./../endeffector/bin/endeffector.o

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

all: $(BINDIR)/$(TARGET)

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	$(CXX) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

$(BINDIR)/$(TARGET): $(OBJECTS)
	$(LINKER) $@ $(LFLAGS) $(OBJECTS) $(LINK_OBJ)
	@echo "Linking complete!"

clean:
	rm -f bin/$(TARGET)
	rm -f obj/*.o
