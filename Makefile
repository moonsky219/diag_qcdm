all: diag_qcdm

diag_qcdm:
	ndk-build
	cp ../libs/armeabi/diag_qcdm .
	rm -r ../obj/ ../libs/

clean:
	rm diag_qcdm
