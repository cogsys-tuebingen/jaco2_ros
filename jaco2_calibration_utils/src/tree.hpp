#ifndef TREE_HPP
#define TREE_HPP

#include <iostream>
#include <memory>
#include <vector>
#include <array>


template<std::size_t Level, std::size_t NodeSize>
struct Node
{
    using Index = std::array<std::size_t, Level>;

    std::array<Node<Level-1ul, NodeSize>, NodeSize> children;
    void print(std::string buff = "")
    {
        for(std::size_t i = 0 ; i < NodeSize ; ++i)
            children[i].print(buff + std::to_string(i) + ",");
    }

    void getIndeces(std::vector<std::vector<int>>& result, std::vector<int>& id)
    {
        for(std::size_t i = 0; i < NodeSize; ++i){
            id.push_back(i);
            children[i].getIndeces(result, id);
        }

        id.pop_back();

    }

    static std::vector<std::vector<int>> getIndeces(Node& node)
    {
        std::vector<int> id;
        std::vector<std::vector<int>> res;
        node.getIndeces(res, id);
        return res;
    }
};

template<std::size_t NodeSize>
struct Node<0, NodeSize> {
    double data;

    std::vector<int> id_;
    Node()
    {
        data = drand48();
    }

    void print(std::string buff)
    {
        std::cout << buff /*<< data << ";"*/ << std::endl;
    }

    void getIndeces(std::vector<std::vector<int>>& result, std::vector<int>& i)
    {
        result.push_back(i);
            i.pop_back();
    }
};


#endif // TREE_HPP
