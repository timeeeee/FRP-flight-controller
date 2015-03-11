all:
	ghc main.hs constants.hs state_estimation.hs

clean:
	rm -f main *.hi *.o *~
