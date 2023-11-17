#pragma once
# include <vector>

namespace linkml{

    struct reg
    {
        reg(reg &r){
            mask.reserve(r.mask.size());
            mask.assign(r.mask.size(),0);
        }
        reg(int size)
        {
            mask.reserve(size);
            mask.assign(size,0);
        }
        std::vector<int> mask;
        std::vector<int> indecies;

        void update(){

            std::vector<int> compacted = std::vector<int>();

            // pragma omp parallel for
            for (size_t i = 0; i< mask.size(); i++)
                if ( mask.at(i) == 1  )
                    compacted.push_back(i);

            indecies = compacted;
        
        }
    };
}
