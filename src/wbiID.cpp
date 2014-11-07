/*
 * Copyright (C) 2013  CoDyCo Consortium
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
 * Authors: Silvio Traversaro, Andrea Del Prete, Marco Randazzo
 * email: silvio.traversaro@iit.it - andrea.delprete@iit.it - marco.randazzo@iit.it
 */

#include "wbiID.h"
#include <sstream>

namespace wbi {

wbiId::wbiId()
{
    id = "WBI_ID_DEFAULT_VALUE";
}

wbiId::wbiId(const std::string & _id)
{
    id = _id;
}

wbiId::wbiId(const char * _id)
{
    id = std::string(_id);
}


const std::string & wbiId::toString() const
{
    return id;
}

wbiId & wbiId::operator=(const wbiId & id_copy)
{
    this->id = id_copy.toString();
    return *this;
}

bool wbiId::operator==(const wbiId & comparison_id) const
{
    return (comparison_id.toString() == this->id);
}


wbiIdList::wbiIdList()
{}
wbiIdList::wbiIdList(const wbiId &id0)
{ addId(id0); }
wbiIdList::wbiIdList(const wbiId &id0, const wbiId &id1)
{ addId(id0); addId(id1);}
wbiIdList::wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2)
{ addId(id0); addId(id1); addId(id2); }
wbiIdList::wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3)
{ addId(id0); addId(id1); addId(id2); addId(id3); }
wbiIdList::wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3, const wbiId &id4)
{ addId(id0); addId(id1); addId(id2); addId(id3); addId(id4); }
wbiIdList::wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3, const wbiId &id4, const wbiId& id5)
{ addId(id0); addId(id1); addId(id2); addId(id3); addId(id4); addId(id5); }
wbiIdList::wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3, const wbiId &id4, const wbiId& id5, const wbiId & id6)
{ addId(id0); addId(id1); addId(id2); addId(id3); addId(id4); addId(id5); addId(id6); }

wbiIdList::wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2)
{ addIdList(lid1); addIdList(lid2); }
wbiIdList::wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); }
wbiIdList::wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3, const wbiIdList &lid4)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); addIdList(lid4); }
wbiIdList::wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3, const wbiIdList &lid4, const wbiIdList &lid5)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); addIdList(lid4); addIdList(lid5); }
wbiIdList::wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3, const wbiIdList &lid4, const wbiIdList &lid5, const wbiIdList &lid6)
{ addIdList(lid1); addIdList(lid2); addIdList(lid3); addIdList(lid4); addIdList(lid5); addIdList(lid6); }

wbiIdList::~wbiIdList() {}

void wbiIdList::pushId(const wbiId &id)
{
    storage.push_back(id);
}

bool wbiIdList::wbiIdToNumericId(const wbiId &_wbiId, int & numericId) const
{
    wbiId wbi_id = _wbiId;
    std::vector<wbiId>::const_iterator it = std::find(storage.begin(),storage.end(),wbi_id);
    if( it == storage.end() )
    {
        return false;
    }
    numericId = std::distance(storage.begin(),it);
    return true;
}

/** Convert a global id into a local id */
bool wbiIdList::numericIdToWbiId(const int numeridId, wbiId & wbi_id) const
{
    if( numeridId < 0 && numeridId >= this->size() )
    {
        return false;
    }
    wbi_id = storage[numeridId];
    return true;
}

//Remove an existing element
bool wbiIdList::removeId(const wbiId &j)
{
    std::vector<wbiId>::iterator it = std::find(storage.begin(),storage.end(),j);
    if( it == storage.end() )
    {
        return false;
    }
    storage.erase(it);
    return true;
}

bool wbiIdList::addId(const wbiId &i)
{
    if(containsId(i))
        return true;
    pushId(i);
    return true;
}

int wbiIdList::addIdList(const wbiIdList &addedList)
{
    int count = 0;
    wbiId added_id;
    for(int i=0; i < addedList.size(); i++ )
    {
        addedList.numericIdToWbiId(i,added_id);
        if(!containsId(added_id))
        {
            pushId(added_id);
            count++;
        }
    }
    return count;
}

bool wbiIdList::containsId(const wbiId &i) const
{
    std::vector<wbiId>::const_iterator it = std::find(storage.begin(),storage.end(),i);
    if(it==storage.end()) {
        return false;
    } else {
        return true;
    }
}

// Get the number of ids in this list
unsigned int wbiIdList::size() const
{
    return storage.size();
}

std::string wbiIdList::toString() const
{
    std::ostringstream s;
    for(int i=0; i < storage.size(); i++ )
    {
        s << storage[i].toString() << std::endl;
    }
    return s.str();
}

}
