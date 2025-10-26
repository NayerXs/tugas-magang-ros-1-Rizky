#include "ros/ros.h"
#include "meine_package/AddBook.h"
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "liblary_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<meine_package::AddBook>("add_book_service");

    meine_package::AddBook srv;

    std::cout << "Enter ISBN: ";
    std::cin >> srv.request.isbn;
    std::cin.ignore();
    std::cout << "Enter Title: ";
    std::getline(std::cin, srv.request.title);
    std::cout << "Enter Author: ";
    std::getline(std::cin, srv.request.author);

    if (client.call(srv)) {
        std::cout << "Server Response: " << srv.response.status << std::endl;
    } else {
        std::cout << "Failed to call service!" << std::endl;
    }

    return 0;
}
