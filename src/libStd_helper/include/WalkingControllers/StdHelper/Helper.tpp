template<typename T>
bool WalkingControllers::StdHelper::appendVectorToDeque(const std::vector<T>& input, std::deque<T>& output, const size_t& initPoint)
{
    if(initPoint > output.size())
    {
        std::cerr << "[StdHelper::appendVectorToDeque] The init point has to be less or equal to the size of the output deque."
                  << std::endl;
        return false;
    }

    // resize the deque
    output.resize(input.size() + initPoint);

    // Advances the iterator it by initPoint positions
    typename std::deque<T>::iterator it = output.begin();
    std::advance(it, initPoint);

    // copy the vector into the deque from the initPoint position
    std::copy(input.begin(), input.end(), it);

    return true;
}
