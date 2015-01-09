/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

#ifndef TG_CAST_H
#define TG_CAST_H

/**
 * @file tgCast.h
 * @brief Utility class for class casting and filtering collections by type 
 * @date March 21, 2014
 * $Id$
 */

// This application
#include "tgTagSearch.h"
#include "tgTaggable.h"
// The C++ Standard Library
#include <vector>

/**
 * Utility class for typecasting 
 */
class tgCast
{
public:
    
    /**
     * Filter by type, e.g. 
     * std::vector<SomeBaseType> myVector;
     * tgCast::filter<SomeBaseType, SomeSubType>(myVector); will return a 
     * vector containing only the elements in myVector that are castable to 
     * SomeSubType
     */
    template <typename T_FROM, typename T_TO>
    static std::vector<T_TO*> filter(const std::vector<T_FROM*>& v)
    {
        std::vector<T_TO*> result;
        for(int i = 0; i < v.size(); i++) {
            T_TO* t = cast<T_FROM, T_TO>(v[i]);
            if(t != 0) {
                result.push_back(t);
            }
        }
        return result;
    }
    
    /**
     * A version of filter that returns const objects inside the vector
     * Filter by type, e.g. 
     * std::vector<SomeBaseType> myVector;
     * tgCast::filter<SomeBaseType, const SomeSubType>(myVector); will return a 
     * vector containing only the elements in myVector that are castable to 
     * const SomeSubType
     */
    template <typename T_FROM, typename T_TO>
    static std::vector<const T_TO*> constFilter(const std::vector<T_FROM*>& v)
    {
        std::vector<const T_TO*> result;
        for(int i = 0; i < v.size(); i++) {
            const T_TO* t = cast<T_FROM, T_TO>(v[i]);
            if(t != 0) {
                result.push_back(t);
            }
        }
        return result;
    }
    
    /**
     * Attempt a dynamic cast to the provided type. If it fails, return 0
     */ 
    template <typename T_FROM, typename T_TO>
    static T_TO* cast(T_FROM* obj)
    {
        // @todo: dynamic_cast may just return 0 on fail...
        try {
            return dynamic_cast<T_TO*>(obj);
        } catch (std::exception& e) {
            return 0;
        }
    }

    /**
     * Attempt a dynamic cast to the provided type. If it fails, return 0
     * This method is designed for const pointers
     */ 
    template <typename T_FROM, typename T_TO>
    static const T_TO* cast(const T_FROM* obj)
    {
        // @todo: dynamic_cast may just return 0 on fail...
        try {
            return dynamic_cast<const T_TO*>(obj);
        } catch (std::exception& e) {
            return 0;
        }
    }

    /**
     * Attempt a dynamic cast to the provided type. If it fails, return 0
     */ 
    template <typename T_FROM, typename T_TO>
    static T_TO* cast(T_FROM& obj)
    {
        return cast<T_FROM, T_TO>(&obj);
    }
    
    template <typename T_FROM, typename T_TO>
    static std::vector<T_TO*> find(const tgTagSearch& tagSearch, const std::vector<T_FROM*> haystack)
    {
        // Filter to the correct type
        std::vector<T_TO*> filtered = filter<T_FROM, T_TO>(haystack);
        std::vector<T_TO*> result;

        // Check each element in filtered to see if it is castable to tgTaggable and matches the search
        for(int i = 0; i < filtered.size(); i++) {
            tgTaggable* t = cast<T_TO, tgTaggable>(filtered[i]);
            if(t != 0 && tagSearch.matches(*t)) {
                // Note: add filtered[i] instead of t to maintain correct type
                result.push_back(filtered[i]);
            }
        }
        return result;
    }
    
};


#endif
