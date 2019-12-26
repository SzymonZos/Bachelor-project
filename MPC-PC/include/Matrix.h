#pragma once
#include <cstdint>
#include <iostream>
#include <fstream>
#include <initializer_list>

class CMatrix
{
protected:
    uint32_t rows;
    uint32_t columns;
    double** matrix;
private:
    void Add(const CMatrix& m);
    void Sub(const CMatrix& m);
    [[nodiscard]] CMatrix Mul(const CMatrix& m) const;
    void Mul(const double& scalar);
    /**
     * Metoda dzielenia przez skalar
     * \param[in] scalar Warto�� przez kt�r� dzielimy kom�rki macierzy
     * \exeption<invalid_argument> { Metoda wyrzuca wyj�tek invalid_argument gdy parametry metody s� niepoprawne }
     */
	void Div(const double& scalar);
public:
    CMatrix() = delete;
    /**
     * Konstruktor pustej macierzy
     * \param[in] rows      Ilo�� wierszy
     * \param[in] columns   Ilo�� kolumn
     */
    CMatrix(uint32_t rows, uint32_t columns);
    /**
     * Konstruktor pustej macierzy
     * \param[in] rows      Ilo�� wierszy
     * \param[in] columns   Ilo�� kolumn
     * \param[in] mat       Wska�nik na tablic� (rows) adres�w tablic (columns) z warto�ciami inicjalizuj�cymi kom�rki macierzy
     */
    CMatrix(uint32_t rows, uint32_t columns, double** mat);
    /**
     * Konstruktor pustej macierzy
     * \param[in] rows      Ilo�� wierszy
     * \param[in] columns   Ilo�� kolumn
     * \param[in] mat       Wska�nik na tablic� (rows x columns) element�w inicjalizuj�cych kom�rki macierzy
     * \parblock
     * Przyk�ad wywo�ania:
     * double mat[3][2] = { { 1, 2 }, { 4, 5 }, { 7, 8 } };
	 * CMatrix m(3, 2, *mat);
     * \endparblock
     */
    CMatrix(uint32_t rows, uint32_t columns, const double* mat);
    CMatrix(const CMatrix& M);

    CMatrix(uint32_t rows, uint32_t columns, std::initializer_list<double> list);

    CMatrix& operator = (const CMatrix& m);
    ~CMatrix();

	void inverse();
	void GetSize(uint32_t& rows, uint32_t& columns) const;
    [[nodiscard]] double& GetElement(uint32_t row, uint32_t column) const;

    virtual double* operator[](uint32_t row) const;

    CMatrix operator+ (const CMatrix& m) const;
    CMatrix& operator+= (const CMatrix& m);

    CMatrix operator- (const CMatrix& m) const;
    CMatrix& operator-= (const CMatrix& m);

    CMatrix operator* (const CMatrix& m) const;
    CMatrix& operator*= (const CMatrix& m);

    CMatrix operator* (const double& scalar) const;
    CMatrix& operator*= (const double& scalar);

	CMatrix operator/ (const double& scalar) const;
	CMatrix& operator/= (const double& scalar);

    CMatrix operator-() const;
    [[nodiscard]] CMatrix T() const;

    CMatrix operator()();
    CMatrix operator()(uint32_t rows, uint32_t columns, const double* mat);

    friend std::ostream& operator<< (std::ostream& stream, const CMatrix& m);
};

class CVector : public CMatrix
{
public:
	explicit CVector(uint32_t columns);
	CVector(uint32_t rows, uint32_t column);
	CVector(uint32_t columns, double* mat);
    CVector(uint32_t rows, uint32_t flag, double* mat);
	[[nodiscard]] CVector Tv() const;
    CVector& operator= (const CMatrix& m);
};