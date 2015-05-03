all:
	ghc main.hs constants.hs state_estimation.hs matrix_ops.hs common_equations.hs state_functions.hs

clean:
	rm -f main *.hi *.o *~
