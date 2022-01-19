test: test.o nes_cpu.o
	g++ -o test test.o nes_cpu.o

test.o: test.cpp nes_cpu.h
	g++ -std=c++11 -c test.cpp

nes_cpu.o: nes_cpu.cpp nes_cpu.h
	g++ -std=c++11 -c nes_cpu.cpp

clean:
	rm *.o test 
