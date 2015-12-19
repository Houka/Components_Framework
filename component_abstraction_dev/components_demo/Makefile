SOURCE=test.cpp ComponentGraphManager.cpp component.cpp ROS_PARAM.cpp
MYPROGRAM=test

CC=g++ -std=c++11

all: $(MYPROGRAM)



$(MYPROGRAM): $(SOURCE)

	$(CC) $(SOURCE) -o$(MYPROGRAM) 

clean:

	rm -f $(MYPROGRAM)