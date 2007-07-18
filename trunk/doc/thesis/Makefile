# This Makefile and the related Latex documents are assembled for
# our students at
# Microelectronic Systems Group, Darmstadt University of Technology
# 
# Please, make sure that your documentation can be reproduced correctly
# on our computers here, aka, this Makefile runs without errors or severe
# warnings!
#
all: pdf

dvi:
	latex thesis.tex
	bibtex thesis
	latex thesis.tex

ps: dvi
	dvips -t a4 thesis.dvi

pdf: ps
	#ps2pdf thesis.ps

thisarchive:
	tar cf thesis_template_zipf.tar titelpage.tex thesis.tex thesis.bib decls_de.tex decls_en.tex abbrev.tex introduction.tex mainpart.tex conclusion.tex athene.eps images Makefile
	gzip thesis_template_zipf.tar
	mv thesis_template_zipf.tar.gz ..

# use with extreme(!) care
clean:
	rm -f *.aux *.bbl *.blg *.lof *.log *.lot *.toc

