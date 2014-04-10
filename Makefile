.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen

############################################

# Project Paths
PROJECT_ROOT := $(CURDIR)
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
LATEX  = cs296_project_report
# Target
TARGET 		= cs296_01_exe

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing -fPIC
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OLS  := $(filter-out $(OBJDIR)/main.o,$(OBJS))

.PHONY: setup exe clean distclean static dynamic exelib doc report

setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin
	@if ! test -e $(BOX2D_ROOT)/lib/libBox2D.a && ! test -e $(BOX2D_ROOT)/include/Box2D; \
	then $(ECHO) "Box2D not Found....." && $(ECHO) "Installing Box2D...." && cd $(BOX2D_ROOT)/src \
	&& tar -xzf Box2D.tgz && cd Box2D && mkdir build296 && cd build296 && cmake ../ && make -s install && make -s && $(ECHO) "Box2D installed."; \
	else $(ECHO) "Box2D Found....";\
	fi;

exe: setup $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(TARGET)"
	@$(CC) -o $(BINDIR)/$(TARGET) $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err


doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"
clean:
	@$(ECHO) -n "Cleaning up...."
	@$(RM) -rf obj bin
	@$(RM) -rf $(DOCDIR)/html
	@$(RM) -rf $(DOCDIR)/$(LATEX).aux $(DOCDIR)/$(LATEX).bbl $(DOCDIR)/$(LATEX).log $(DOCDIR)/$(LATEX).dvi $(DOCDIR)/$(LATEX).pdf $(DOCDIR)/$(LATEX).blg
	@$(ECHO) "Done"
	
report:
	@cd $(DOCDIR); pdflatex $(LATEX).tex; bibtex $(LATEX); pdflatex $(LATEX).tex; pdflatex $(LATEX).tex; 
distclean: clean
	@cd $(BOX2D_ROOT) && $(RM) -rf include/* src/Box2D lib/*
