#pragma once
#pragma warning( disable : 4290 ) 
#include <cstdint>
#include <iostream>
#include <fstream>

using namespace std;

class CMatrix
{
	friend class aproxMeanSquare;
protected:
    uint32_t rows;
    uint32_t columns;
    double **matrix;
private:
    void Add(const CMatrix &m) throw(invalid_argument);
    void Sub(const CMatrix &m) throw(invalid_argument);
    CMatrix Mul(const CMatrix &m) const throw(invalid_argument);
    void Mul(const double &scalar) throw(invalid_argument);
    /**
     * Metoda dzielenia przez skalar
     * \param[in] scalar Wartoœæ przez któr¹ dzielimy komórki macierzy
     * \exeption<invalid_argument> { Metoda wyrzuca wyj¹tek invalid_argument gdy parametry metody s¹ niepoprawne }
     */
	void Div(const double &scalar) throw(invalid_argument);
public:
    /**
     * Konstruktor pustej macierzy
     * \param[in] rows      Iloœæ wierszy
     * \param[in] columns   Iloœæ kolumn
     */
    CMatrix(uint32_t rows, uint32_t columns);
    /**
     * Konstruktor pustej macierzy
     * \param[in] rows      Iloœæ wierszy
     * \param[in] columns   Iloœæ kolumn
     * \param[in] mat       WskaŸnik na tablicê (rows) adresów tablic (columns) z wartoœciami inicjalizuj¹cymi komórki macierzy
     */
    CMatrix(uint32_t rows, uint32_t columns, double **mat);
    /**
     * Konstruktor pustej macierzy
     * \param[in] rows      Iloœæ wierszy
     * \param[in] columns   Iloœæ kolumn
     * \param[in] mat       WskaŸnik na tablicê (rows x columns) elementów inicjalizuj¹cych komórki macierzy
     * \parblock
     * Przyk³ad wywo³ania:
     * double mat[3][2] = { { 1, 2 }, { 4, 5 }, { 7, 8 } };
	 * CMatrix m(3, 2, *mat);
     * \endparblock
     */
    CMatrix(uint32_t rows, uint32_t columns, double *mat);
    CMatrix(const CMatrix & M);
    CMatrix & operator = (const CMatrix &m);
    ~CMatrix();

	void inverse();
	void zero();
	void GetSize(uint32_t &rows, uint32_t &columns) const;
    double &GetElement(uint32_t row, uint32_t column) throw(invalid_argument);
    double *operator[](uint32_t row) throw(invalid_argument);

    CMatrix operator + (const CMatrix &m) const;
    CMatrix & operator += (const CMatrix &m);

    CMatrix operator - (const CMatrix &m) const;
    CMatrix & operator -= (const CMatrix &m);

    CMatrix operator * (const CMatrix &m) const;
    CMatrix & operator *= (const CMatrix &m);

    CMatrix operator * (const double &scalar) const;
    CMatrix & operator *= (const double &scalar);

	CMatrix operator / (const double &scalar) const;
	CMatrix & operator /= (const double &scalar);

    CMatrix operator -() const;
    CMatrix T() const;

    friend ostream& operator<< (ostream &stream, const CMatrix &m);

	void save();
};

class CVector : public CMatrix
{
public:
	CVector(uint32_t columns);
	CVector(uint32_t columns, uint32_t flag);
	CVector(uint32_t columns, double *mat);
	void SetMatrixColumn(uint32_t column, CMatrix &m);
	CVector Tv() const;
    CVector & operator = (const CMatrix &m);
};