all: paper

paper: 
	#-rm img/*eps-converted-to.pdf
	pdflatex scaling_counterexample.tex
	bibtex scaling_counterexample
	pdflatex scaling_counterexample.tex
	pdflatex scaling_counterexample.tex

clean:
	-rm *.aux
	-rm *.log
	-rm *.bbl
	-rm scaling_counterexample.pdf
	-rm scaling_counterexample.dvi
	-rm scaling_counterexample.ps
	-rm *.blg
	-rm img/*eps-converted-to.pdf
	-rm *~
