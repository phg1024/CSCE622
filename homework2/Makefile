all: p1 p2 p3

p1:
	g++ -std=c++11 -g p1_default_ctr.cpp -o p1_default_ctr &
	g++ -std=c++11 -g p1_copy_ctr.cpp -o p1_copy_ctr &
	g++ -std=c++11 -g p1_copy_assign_op.cpp -o p1_copy_assign_op &
	g++ -std=c++11 -g p1_equality_op.cpp -o p1_equality_op
	g++ -std=c++11 -g p1_inequality_op.cpp -o p1_inequality_op &
	g++ -std=c++11 -g p1_accessor.cpp -o p1_accessor

p2:
	g++ -std=c++11 -O2 p2_test.cpp -o p2_test

p3:
	g++ -std=c++11 -O2 p3.cpp -o p3

p4:
	g++ -std=c++11 -O2 p4.cpp -o p4

clean:
	rm p1_default_ctr p1_copy_ctr p1_copy_assign_op p1_equality_op p1_inequality_op p1_accessor &
	rm p2_test &
	rm p3 &
	rm p4
