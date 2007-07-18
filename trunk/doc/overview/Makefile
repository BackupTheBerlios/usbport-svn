all: main.pdf

main.pdf: main.tex
	latex main.tex
	#dvipdf main.dvi
	xdvi main.dvi


ps:
	dvi2ps main.dvi > main.ps

clean:
	rm *.dvi *.toc *.log *.aux *.pdf

