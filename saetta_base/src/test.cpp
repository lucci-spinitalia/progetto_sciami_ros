#include <functional>
#include <mutex>
#include <thread>
#include <list>
#include <iostream>
#include <sstream>
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <cstdlib>
#include <random>

std::list<std::string> local_messages;
std::mutex msg_mutex;

void pipe_eater (void)
{
	std::string mystr;
	std::stringstream ss;
	while( std::getline(std::cin, mystr) )
	{
		msg_mutex.lock();
		local_messages.push_back(mystr);
		std::cout << "Child: message queue: " << local_messages.size() << std::endl;
		std::cout << "Captured message :\"" << mystr << "\"" << std::endl;
		msg_mutex.unlock();
	}
	
	
}

int main (void)
{
    /*pthread_t th_output;
    pthread_attr_t attr_main;
    pthread_attr_init(&attr_main);
    pthread_create(&th_output, &attr_main, &pipe_eater, NULL);*/
	std::thread t(pipe_eater);
	while(1){
	
	while (local_messages.size() > 0)
	{
		std::cout << "Parent grabbing mutex" << std::endl;
		msg_mutex.lock();
		std::cout << "Queue size: " << local_messages.size() << std::endl;
		std::cout << local_messages.front() << std::endl;
		local_messages.pop_front();
		msg_mutex.unlock();
		std::cout << "Parent releasing mutex" << std::endl << std::endl;
		if(local_messages.size() == 0)
			local_messages.clear();
	}
	}
	t.join();
	return 0;
	


}
