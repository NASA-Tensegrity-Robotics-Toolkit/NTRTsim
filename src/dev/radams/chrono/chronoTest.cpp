#include <iostream>
#include <boost/chrono.hpp>
//#include <chrono>
#include <ctime>
 
long fibonacci(unsigned n)
{
    if (n < 2) return n;
    return fibonacci(n-1) + fibonacci(n-2);
}
 
int main()
{
    boost::chrono::time_point<boost::chrono::system_clock> start, end;
    start = boost::chrono::system_clock::now();
    std::cout << "f(42) = " << fibonacci(42) << '\n';
    end = boost::chrono::system_clock::now();
 
    boost::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = boost::chrono::system_clock::to_time_t(end);
 
    std::cout << "finished computation at " << std::ctime(&end_time)
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
