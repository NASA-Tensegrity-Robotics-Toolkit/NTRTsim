all: paper

paper: 
	#-rm img/*eps-converted-to.pdf
	pdflatex scaling_analysis.tex
	bibtex scaling_analysis
	pdflatex scaling_analysis.tex
	pdflatex scaling_analysis.tex

clean:
	-rm *.aux
	-rm *.log
	-rm *.bbl
	-rm scaling_analysis.pdf
	-rm scaling_analysis.dvi
	-rm scaling_analysis.ps
	-rm *.blg
	-rm img/*eps-converted-to.pdf
	-rm *~
