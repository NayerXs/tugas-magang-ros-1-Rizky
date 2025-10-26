#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include "ros/ros.h"
#include "meine_package/AddBook.h"

using namespace std;

class Book {
private:
    int isbn;
    string title;
    string author;
    bool available;

public:
    Book(int isbn, const string& title, const string& author) 
        : isbn(isbn), title(title), author(author), available(true) {}
    
    int getISBN() const { return isbn; }
    string getTitle() const { return title; }
    string getAuthor() const { return author; }
    bool isAvailable() const { return available; }
    
    void setAvailable(bool avail) { available = avail; }
    
    void display() const {
        cout << "ISBN: " << isbn << ", Title: " << title 
                  << ", Author: " << author << ", Available: " 
                  << (available ? "Yes" : "No") << endl;
    }
};

class Library {
private:
    vector<Book> books;
    
public:
    void addBook(int isbn, const string& title, const string& author) {
        books.emplace_back(isbn, title, author);
        cout << "Book added successfully!\n";
    }
    
    void displayAllBooks() const {
        if (books.empty()) {
            cout << "No books in library.\n";
            return;
        }
        for (const auto& book : books) {
            book.display();
        }
    }
    
    Book* findBook(int isbn) {
        auto it = find_if(books.begin(), books.end(),
            [isbn](const Book& b) { return b.getISBN() == isbn; });
        return (it != books.end()) ? &(*it) : nullptr;
    }
    
    bool updateBook(int isbn, const string& title, const string& author) {
        Book* book = findBook(isbn);
        if (book) {
            cout << "Book updated successfully!\n";
            return true;
        }
        cout << "Book not found!\n";
        return false;
    }
    
    bool removeBook(int isbn) {
        auto it = find_if(books.begin(), books.end(),
            [isbn](const Book& b) { return b.getISBN() == isbn; });
        if (it != books.end()) {
            books.erase(it);
            cout << "Book removed successfully!\n";
            return true;
        }
        cout << "Book not found!\n";
        return false;
    }
};

Library library; 

bool addBookCallback(meine_package::AddBook::Request &req,
                     meine_package::AddBook::Response &res)
{
    library.addBook(req.isbn, req.title, req.author);
    res.status = "Book added successfully!";
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "liblary_server");
    ros::NodeHandle nh;


    // Koneksi Menuju ROS Service
    ros::ServiceServer server = nh.advertiseService("add_book_service", addBookCallback);

    ros::spin();
    return 0;
}