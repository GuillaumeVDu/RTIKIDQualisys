/*
 * Copyright (c) 2016, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef HEADERFILE_H
#define HEADERFILE_H

#include <CommonCEINMS.h>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <sstream>

class HeaderFile
{
	public:
		HeaderFile();
		
		~HeaderFile()
		{
			
		}
		
		void readFile(std::istream& file, const std::string& fileName = "file");
		void writeFile(std::ostream& file, const std::string& fileName = "file", const std::string& firstLine = "default");
		
		inline void setNumberOfRow(const unsigned int& numberOfRow)
		{
			numberOfRow_ = numberOfRow;
		}
		
		inline const unsigned int& getNumberOfRow() const
		{
			return numberOfRow_;
		}
		
		void setNumberOfColumn(const unsigned int& numberOfColumn)
		{
			numberOfColumn_ = numberOfColumn;
		}
		
		const unsigned int& getNumberOfColumn() const
		{
			return numberOfColumn_;
		}
		
		void setNameOfColumn(const std::vector<std::string>& nameOfColumn)
		{
			nameOfColumn_ = nameOfColumn;
		}
		
		const std::vector<std::string>& getNameOfColumn() const
		{
			return nameOfColumn_;
		}
		
		void setInDegrees(const bool& inDegress)
		{
			inDegress_ = inDegress;
		}
		
		inline const bool& getInDegrees() const
		{
			return inDegress_;
		}
		
	protected:

		std::istream& safeGetline(std::istream& is, std::string& t);
		
		bool 				inDegress_;
		std::vector<std::string> 	nameOfColumn_;
		unsigned int 			numberOfColumn_;
		unsigned int 			numberOfRow_;
		bool 				numberOfRowBool_;
		bool 				numberOfColumnBool_;
};

#endif // HEADERFILE_H
