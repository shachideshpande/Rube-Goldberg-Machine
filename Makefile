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
######################################
# Project Name (generate executable with this name)
TARGET = cs251_base

# Project Paths
PROJECT_ROOT=./
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
CPPFLAGSPG =-pg -O3 -Wall -fno-strict-aliasing

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
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


.PHONY: all setup doc clean distclean report codeDoc release profileobj profile

all: setup release



setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin

b2dsetup:
	@if [ ! -d "$(EXTERNAL_ROOT)/src/Box2D" ]; \
	then $ tar zxf $(EXTERNAL_ROOT)/src/Box2D.tgz -C $(EXTERNAL_ROOT)/src; \
	else $(ECHO) "Box2D is already installed..."; \
	fi;
	@cd $(EXTERNAL_ROOT); \
	cd src; \
	cd Box2D; \
	mkdir build251; \
	cd build251; \
	cmake -DCMAKE_BUILD_TYPE=Release ../; \
	make; \
	make install; \

release: setup b2dsetup $(OBJS) 
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -pg -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err
	@mv release ./bin

-include -include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -pg -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err



clean:	
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(OBJDIR) *~ $(DEPS) $(SRCDIR)/*~
	@$(ECHO) "Done"

codeDoc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

report:
	@cd ./doc
	@cp ./doc/*.bib ./
	@pdflatex ./doc/ProjectReport.tex > tp.log
	@echo "Step 1st done..."
	
	@bibtex ProjectReport > tp.log
	
	@echo "Step 2nd done..."	
	@pdflatex ./doc/ProjectReport.tex > tp.log
	@echo "Step 3rd done..."	
	@pdflatex ./doc/ProjectReport.tex > tp.log
	@echo "report done..."	
	@cd ./doc 
	@pdflatex ./doc/BeamerPresentation.tex > tp.log	
	@echo "presentation done..."
	@mv *.pdf ./doc
	@rm -f *.log *.out *.aux *.nav 
	@rm -f *.snm *.toc *.bbl *.blg
	@rm -f *.bib	
	@rm -f ./doc/*.aux 
	@rm -f ./doc/ProjectReport.blg 





distclean:clean
	@$(RM) -rf $(BINDIR) $(DOCDIR)/html
	@cd $(DOCDIR)
	@rm -f ./doc/*.pdf 2> tp.log
	@rm -f ./doc/analysis.txt 2> tp.log
	@rm -f ./doc/profile.png	
	@rm -f tp.log
	@$(RM) -rf ./external/lib/*
	@$(RM) -rf ./external/include/*
	@$(RM) -rf ./external/src/Box2D
	@$(RM) -rf gmon.out

profile: all
	@./bin/release	
	@gprof ./bin/release gmon.out  > analysis.txt
	@python3 ./doc/gprof2dot.py analysis.txt >callgraph.dot
	@dot -Tpng callgraph.dot > profile.png	
	@mv *.png *.txt ./doc/
	@rm -f callgraph.dot 

-include -include $(OBJS:.o=.d)


	
