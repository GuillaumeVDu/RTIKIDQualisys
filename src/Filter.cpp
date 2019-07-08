/*
 * Copyright 2015 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <Filter.h>

template<typename T>
FilterKin::Filter<T>::Filter(const std::vector<T>& aCoeff, const std::vector<T>& bCoeff):
	_aCoeff(aCoeff), _bCoeff(bCoeff)
{
	for (vectTI it = _bCoeff.begin(); it != _bCoeff.end(); it++)
	{
		_pastData.push_back(0);
		_pastDataFilter.push_back(0);
	}

	_pastDataFilter.pop_back();
}

template<typename T>
FilterKin::Filter<T>::Filter(const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, std::vector<T> pastData ):
	_aCoeff(aCoeff), _bCoeff(bCoeff), _pastData(pastData.begin(), pastData.end()), _pastDataFilter(pastData.begin(), pastData.end())
{
	if (_pastData.size() != _bCoeff.size())
	{
		std::cout << "pastData have size: " << _pastData.size() << " Require: " << _bCoeff.size() << std::endl;
		std::exit(EXIT_FAILURE);
	}
	_pastDataFilter.pop_back();
}

template<typename T>
FilterKin::Filter<T>::~Filter()
{

}

template<typename T>
T FilterKin::Filter<T>::filter(const T& data)
{
	T dataOut = 0;

	_pastData.push_front(data);
	_pastData.pop_back();

	for (vectTCI it = _bCoeff.begin(); it != _bCoeff.end(); it++)
		dataOut += *it * _pastData[std::distance<vectTCI>(_bCoeff.begin(), it)];
	for (vectTCI it = _aCoeff.begin(); it != _aCoeff.end(); it++)
		dataOut -= *it * _pastDataFilter[std::distance<vectTCI>(_aCoeff.begin(), it)];

	_pastDataFilter.push_front(dataOut);
	_pastDataFilter.pop_back();
	
	return dataOut;
}

typedef FilterKin::Filter<float> fFilter;

