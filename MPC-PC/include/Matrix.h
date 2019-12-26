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
     * \param[in] scalar Wartoœæ przez któr¹ dzielimy komórki macierzy
     * \exeption<invalid_argument> { Metoda wyrzuca wyj¹tek invalid_argument gdy parametry metody s¹ niepoprawne }
     */
	void Div(const double& scalar);
public:
    CMatrix() = delete;
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
    CMatrix(uint32_t rows, uint32_t columns, double** mat);
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