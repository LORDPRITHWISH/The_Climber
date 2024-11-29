#include<iostream>
#include<vector>

std::vector<int> vec;

void vecre(){
    for(int i=0;i<10;i++){
        vec.push_back(i);
    }
    std::cout<<"vector size: "<<vec.size()<<'\n';
    std::cout<<"vector begin: "<<vec.front()<<'\n';
    std::cout<<"vector end: "<<vec.back()<<'\n';
}

void vecshow(){
    for(int i=0;i<vec.size();i++){
        std::cout<<vec[i]<<'\n';
    }
}

void detail(){
    std::cout<<"vector size: "<<vec.size()<<'\n';
    std::cout<<"vector begin: "<<vec.front()<<'\n';
    std::cout<<"vector end: "<<vec.back()<<'\n';
}

int main() {
    // std::cout << "Hello, World!" << std::endl;
    vecre();
    // vecshow();
    detail();
    return 0;
}