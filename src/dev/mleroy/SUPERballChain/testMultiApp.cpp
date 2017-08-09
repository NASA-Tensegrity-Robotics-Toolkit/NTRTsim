#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <ncurses.h>

int main(int argc, char** argv)
{
	initscr();
	int c;
	while((c=getch())!=27)
	{
		system("../../../../build/dev/introWorkspace/App3BarYAML ../../../../resources/YamlStructures/NTRT_Intro_Structures/threeBarModel.yaml");
		printw("%d %c\n",c,c);
		refresh();
	}
	endwin();	
q
		std::cout << i << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	std::cout << "Lift off" << std::endl;
	return 0;
}