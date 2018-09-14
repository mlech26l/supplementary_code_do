
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin

TARGET   = bnnopt
TARGET2   = fully_connected

TARGET_LIB = nnpp_lib.o

OPT = -O3
CXX       = clang++
# compiling flags here
CFLAGS   =  -pthread -std=c++11 $(OPT) -Wall -I. -I./../endeffector/src/


LINKER   = clang++ -o
# linking flags here
LFLAGS   = -pthread -Wall $(OPT) -I. -lm

LINK_OBJ = ./../endeffector/bin/endeffector.o

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJECTS_BIN1  :=  $(filter-out $(OBJDIR)/fully_connected.o,$(OBJECTS))
OBJECTS_BIN2  := $(filter-out $(OBJDIR)/bnnopt.o,$(OBJECTS))
MAINS := $(TARGET) $(TARGET2)
LIB_OBJECTS := $(filter-out $(OBJDIR)/$(MAINS).o,$(OBJECTS))

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
	$(LINKER) $@ $(LFLAGS) $(OBJECTS_BIN1) $(LINK_OBJ)
	@echo "Linking complete!"

$(BINDIR)/$(TARGET2): $(OBJECTS)
	$(LINKER) $@ $(LFLAGS) $(OBJECTS_BIN2) $(LINK_OBJ)
	@echo "Linking complete!"

clean:
	rm -f bin/$(TARGET_LIB)
	rm -f bin/testbench
	rm -f bin/$(TARGET)
	rm -f obj/*.o
