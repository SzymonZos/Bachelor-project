NAME  = polsl-msth
SHELL = bash
PWD   = $(shell pwd)
VERS  = $(shell ltxfileinfo -v $(NAME).dtx)
LOCAL = $(shell kpsewhich --var-value TEXMFLOCAL)
UTREE = $(shell kpsewhich --var-value TEXMFHOME)

VERSION = 0.0.1

PHONY += all
all: dvi

PHONY += dvi
dvi: $(NAME).dvi $(NAME)-example.dvi

PHONY += pdf
pdf: $(NAME).pdf $(NAME)-example.pdf

%.eps: %.png
	convert $< eps3:$@

$(NAME).dvi $(NAME).cls README: $(NAME).dtx pi.eps
	latex -recorder -interaction=batchmode $(NAME).dtx >/dev/null
	if [ -f $(NAME).glo ]; then \
		makeindex -q -s gglo.ist -o $(NAME).gls $(NAME).glo; \
	fi
	if [ -f $(NAME).idx ]; then \
		makeindex -q -s gind.ist -o $(NAME).ind $(NAME).idx; \
	fi
	latex --recorder --interaction=nonstopmode $(NAME).dtx > /dev/null
	latex --recorder --interaction=nonstopmode $(NAME).dtx > /dev/null
	test -e README.txt && mv README.txt README || exit 0

$(NAME)-example.dvi: $(NAME)-example.tex $(NAME).cls pi.eps
	latex -recorder -interaction=batchmode $(NAME)-example.tex >/dev/null
	if [ -f $(NAME)-example.glo ]; then \
		makeindex -q -s gglo.ist -o $(NAME)-example.gls $(NAME).glo; \
	fi
	if [ -f $(NAME)-example.idx ]; then \
		makeindex -q -s gind.ist -o $(NAME)-example.ind $(NAME).idx; \
	fi
	latex --recorder --interaction=nonstopmode $(NAME)-example.tex > /dev/null
	latex --recorder --interaction=nonstopmode $(NAME)-example.tex > /dev/null

$(NAME)-example.pdf: $(NAME)-example.tex $(NAME).cls
	pdflatex -recorder -interaction=batchmode $(NAME)-example.tex >/dev/null
	if [ -f $(NAME)-example.glo ]; then \
		makeindex -q -s gglo.ist -o $(NAME)-example.gls $(NAME).glo; \
	fi
	if [ -f $(NAME)-example.idx ]; then \
		makeindex -q -s gind.ist -o $(NAME)-example.ind $(NAME).idx; \
	fi
	pdflatex --recorder --interaction=nonstopmode $(NAME)-example.tex > /dev/null
	pdflatex --recorder --interaction=nonstopmode $(NAME)-example.tex > /dev/null

$(NAME).pdf: $(NAME).dtx
	pdflatex -recorder -interaction=batchmode $(NAME).dtx >/dev/null
	if [ -f $(NAME).glo ]; then makeindex -q -s gglo.ist -o $(NAME).gls $(NAME).glo; fi
	if [ -f $(NAME).idx ]; then makeindex -q -s gind.ist -o $(NAME).ind $(NAME).idx; fi
	pdflatex --recorder --interaction=nonstopmode $(NAME).dtx > /dev/null
	pdflatex --recorder --interaction=nonstopmode $(NAME).dtx > /dev/null

PHONY += clean
clean:
	rm -f $(NAME).{aux,fls,glo,gls,hd,idx,ilg,ind,ins,log,out}
	rm -f $(NAME)-example.{aux,fls,glo,gls,hd,idx,ilg,ind,ins,log,out}

PHONY += distclean
distclean: clean
	rm -f $(NAME).{dvi,cls} README $(NAME)-example.pdf

IMAGES = pi.png pi.eps

PHONY += inst
inst: all
	mkdir -p $(UTREE)/{tex,source,doc}/latex/$(NAME)
	cp $(NAME).dtx $(UTREE)/source/latex/$(NAME)
	cp $(NAME).cls $(IMAGES) $(UTREE)/tex/latex/$(NAME)
	cp $(NAME).dvi $(UTREE)/doc/latex/$(NAME)

PHONY += install
install: all
	mkdir -p $(LOCAL)/{tex,source,doc}/latex/$(NAME)
	install -m 644 $(NAME).dtx $(LOCAL)/source/latex/$(NAME)
	install -m 644 $(NAME).cls $(IMAGES) $(LOCAL)/tex/latex/$(NAME)
	install -m 644 $(NAME).dvi $(LOCAL)/doc/latex/$(NAME)

PHONY += help
help: FORCE
	@echo "Targets:"
	@echo "* all           - build "
	@echo "  inst          - install package to text home directory"
	@echo "  install       - install package to TEXMFLOCAL directory"
	@echo "  distclean     - remove all generated files"
	@echo "  clean         - remove most generated files"

PHONY += FORCE
FORCE:

PHONY += dist
dist:
	git archive --format tar.gz HEAD --prefix=$(NAME)-$(VERSION)/ \
		--output $(NAME)-$(VERSION).tar.gz

.PHONY: $(PHONY)
