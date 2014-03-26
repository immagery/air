#ifndef SYM_MATRIX_H
#define SYM_MATRIX_H

#define MATRIX_PRECISION double

class symMatrix
{
public:
	symMatrix()
	{
		size = 0;
		withDiagonal = true;
		elements.clear();
	}

	~symMatrix()
	{
		size = 0;
		withDiagonal = true;
		for(unsigned int i = 0; i< elements.size(); i++)
			elements[i].clear();
		elements.clear();
	}

	void resize(int _size, bool _withDiagonal = true)
	{
		size = _size;
		withDiagonal = _withDiagonal;
		elements.resize(size);

		for(int i = 0; i< size; i++)
		{
			if(withDiagonal)
				elements[i].resize(size-i, 0);
			else
				elements[i].resize(size-i-1, 0);
		}
	}

	double getSafe(int i, int j)
	{
		if(i < size && i > 0 && j < size && j > 0)
		{
			if(j <= i)
				return elements[j][i-j];
			else /*if(j > i)*/
				return elements[i][j-i];
		}

		return 0;
	}

	double get(int i, int j)
	{
		if(j <= i)	
			return elements[j][i-j];
		else /*if(j > i)*/
				return elements[i][j-i];
	}
	
	void setSafe(int i, int j, double value)
	{
		if(i < size && i > 0 && j < size && j > 0)
		{
			if(j <= i)	
				elements[j][i-j] = value;
			else if(j > i)
				elements[i][j-i] = value;
		}
	}

	void set(int i, int j, double value)
	{
		if(j <= i)	
			elements[j][i-j] = value;
		else if(j > i)
			elements[i][j-i] = value;
	}

	vector< vector<double> > elements;
	int size;
	bool withDiagonal;
};

class symMatrixLight
{
public:
	symMatrixLight()
	{
		size = 0;
		sizeShort = 0;
		withDiagonal = true;
		elements = NULL;
	}

	~symMatrixLight()
	{
		size = 0;
		sizeShort = 0;
		withDiagonal = true;
		free(elements);
	}

	void resize(int _size, bool _withDiagonal = true)
	{
		size = _size;
		sizeShort = ceil(_size/2);
		withDiagonal = _withDiagonal;
		
		elements = (MATRIX_PRECISION*) malloc(sizeof(MATRIX_PRECISION) * size * size);

		int length = size * size;
		for(int i = 0; i< length; i++) elements[i] = 0;
	}

	double getSafe(int i, int j)
	{
		if(i < size && i > 0 && j < size && j > 0)
		{
			if(j <= i)
				return elements[j*size + (i-j)];
			else /*if(j > i)*/
				return elements[i*size + (j-i)];
		}

		return 0;
	}

	double get(int i, int j)
	{
		if(j <= i)	
			return elements[j*size + (i-j)];
		else /*if(j > i)*/
				return elements[i*size + (j-i)];
	}
	
	void setSafe(int i, int j, double value)
	{
		if(i < size && i > 0 && j < size && j > 0)
		{
			if(j <= i)	
				elements[j*size + (i-j)] = value;
			else if(j > i)
				elements[i*size + (j-i)] = value;
		}
	}

	void set(int i, int j, double value)
	{
		if(j <= i)	
			elements[j*size + (i-j)] = value;
		else if(j > i)
			elements[i*size + (j-i)] = value;
	}


	MATRIX_PRECISION* elements;
	int size;
	int sizeShort;
	bool withDiagonal;
};

#endif // SYM_MATRIX_H