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
 * Authors: Silvio Traversaro, Andrea Del Prete
 * email: silvio.traversaro@iit.it - andrea.delprete@iit.it
 */

#ifndef WBI_ID_H
#define WBI_ID_H

#include <vector>
#include <string>

namespace wbi
{
    
    /**
     * Identifier used by wholeBodyInterface. It is the class used to identify uniquely
     * joints, sensors, links and every other kind of entity in wholeBodyInterface .
     *
     */
    class ID
    {
    private:
        std::string id;
    public:
        ID();
        
        ID(const std::string & new_id);
        
        ID(const char * new_id);
        
        
        ID & operator=(const ID & id_copy);
        
        /**
         * Get the identifier string for the ID
         */
        const std::string & toString() const;
        
        bool operator==(const ID & comparison_id) const;
    };
    
    /**
     * List of wholeBodyInterface identifiers (ID).
     * Used to represent lists of joints, sensors, links, ... .
     */
    class IDList
    {
    private:
        std::vector<ID> storage;
        
    protected:
        /** Add the specified id without checking its existance. */
        void pushID(const ID & id);
        
    public:
        IDList();
        IDList(const ID &id0);
        /**
         * Create an id list with the specified ids.
         * @param id0 First id
         * @param id1 Second id
         */
        
        /** \note this helper function should be deprecated
         *        as they facilitate putting IDList in code
         *        while they should be in configuration files.
         */
        IDList(const ID &id0, const ID &id1);
        IDList(const ID &id0, const ID &id1, const ID &id2);
        IDList(const ID &id0, const ID &id1, const ID &id2, const ID &id3);
        IDList(const ID &id0, const ID &id1, const ID &id2, const ID &id3, const ID &id4);
        IDList(const ID &id0, const ID &id1, const ID &id2, const ID &id3, const ID &id4, const ID& id5);
        IDList(const ID &id0, const ID &id1, const ID &id2, const ID &id3, const ID &id4, const ID& id5, const ID & id6);
        
        IDList(const IDList &lid1, const IDList &lid2);
        IDList(const IDList &lid1, const IDList &lid2, const IDList &lid3);
        IDList(const IDList &lid1, const IDList &lid2, const IDList &lid3, const IDList &lid4);
        IDList(const IDList &lid1, const IDList &lid2, const IDList &lid3, const IDList &lid4, const IDList &lid5);
        IDList(const IDList &lid1, const IDList &lid2, const IDList &lid3, const IDList &lid4, const IDList &lid5, const IDList &lid6);
        
        ~IDList();
        
        /**
         * Convert a ID to a numeric id.
         * Return the numeric id (index) of the specified ID in this IDList.
         * @return true it the specified ID is found in the list, false otherwise.
         */
        bool wbiIdToNumericId(const ID &wbi_id, int & numericId) const;
        
        /**
         * Convert a numeric id to a ID
         * @return true it the specified ID is found in the list, false otherwise
         */
        bool numericIdToWbiId(const int numeridId, ID & wbi_id) const;
        
        /**
         * Remove the specified id from the list
         * @return true if the id was found and removed, false if it was not found
         */
        bool removeID(const ID &id);
        
        /**
         * Add the specified id to the list.
         * @param id id to add
         * @return true if the id has been added
         */
        bool addID(const ID &id);
        
        /**
         * Add the specified ids to the list.
         *
         * \note this function has a computational complexity of O(n^2),
         *       where n is the maximum between the size of the two used lists.
         *
         * @param appendedList id list to add
         * @return the number of ids added to the list
         *        (0 if all the elements in the appendedList were already in the list,
         *         appendedList.size() if all the element in the appendedList were not in the list.
         *
         */
        int addIDList(const IDList &appendedList);
        
        /**
         * Get the number of ids in this list
         * @return the number of ids in the list
         */
        unsigned int size() const;
        
        /**
         * Check whether the specified id is present in this list.
         * @return true if the specified id is in the list, false otherwise.
         */
        bool containsID(const ID &i) const;
        
        /**
         * Provide a human readable represent of the list
         */
        std::string toString() const;
    };
    
    
} // end namespace

#endif

