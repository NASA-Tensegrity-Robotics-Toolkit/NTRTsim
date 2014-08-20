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

/**
 * @file tgTaggable.h
 * @brief Contains the definition of class tgTaggable
 * @author Ryan Adams
 * $Id$
 */

#ifndef TG_TAGGABLE_H
#define TG_TAGGABLE_H

#include <sstream>
#include <vector>
#include <set>
#include <assert.h>
#include <cstdio>
#include <stdlib.h> //atoi
#include <algorithm>

#include "tgTags.h"

class tgTaggable
{
public:
    
    tgTaggable() {}
    
    tgTaggable(const std::string& space_separated_tags) : m_tags(space_separated_tags) 
    {}

    tgTaggable(tgTags tags) : m_tags(tags)
    {}
    
    ~tgTaggable() {}

    void addTags(const std::string& space_separated_tags)
    {
        m_tags.append(space_separated_tags);
    }

    void addTags(const tgTags& tags)
    {
        m_tags.append(tags);
    }
    
    bool hasTag(const std::string tag) const
    {
        return m_tags.contains(tag);
    }
    
    
    bool hasAllTags(std::string tags)
    {
        return m_tags.contains(tags);
    }
    
    bool hasAnyTags(const std::string tags)
    {
        return m_tags.containsAny(tags);
    }

    bool hasNoTags()
    {
        return m_tags.empty();
    }

    tgTags& getTags()
    {
        return m_tags;
    }

    const tgTags& getTags() const
    {
        return m_tags;
    }
    
    void setTags(tgTags tags) 
    {
        m_tags = tags;
    }

    // @todo: remove this -- tgTags does this...
    std::string getTagStr(std::string delim = " ") const {
        if(m_tags.empty())
            return "";
        std::ostringstream result;
        result << m_tags[0];        
        for(int i = 1; i < m_tags.size(); i++) {
            result << delim << m_tags[i];
        }
        return result.str();
    }
    
private:

    tgTags m_tags;
};

#endif
