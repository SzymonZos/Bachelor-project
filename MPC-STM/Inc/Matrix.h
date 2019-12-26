#pragma once
#include <cstdint>
#include <fstream>
#include <initializer_list>

class CMatrix {

protected:
    uint32_t rows;
    uint32_t columns;
    double** matrix{};

public:
    CMatrix() = delete;
    CMatrix(uint32_t rows, uint32_t columns);
    CMatrix(uint32_t rows, uint32_t columns, double** mat);
    CMatrix(uint32_t rows, uint32_t columns, const double* mat);
    CMatrix(uint32_t rows, uint32_t columns, const std::initializer_list<double>& list);
    CMatrix(const CMatrix& M);
    ~CMatrix();

    void GetSize(uint32_t& rows, uint32_t& columns) const;
    [[nodiscard]] double& GetElement(uint32_t row, uint32_t column) const;

    double* operator[](uint32_t row) const;
    CMatrix& operator= (const CMatrix& m);

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
    CMatrix operator()(uint32_t rows, uint32_t columns, const std::initializer_list<double>& list);

    friend std::ostream& operator<< (std::ostream& stream, const CMatrix& m);

private:
    void make_matrix();
    void Add(const CMatrix& m);
    void Sub(const CMatrix& m);
    [[nodiscard]] CMatrix Mul(const CMatrix& m) const;
    void Mul(const double& scalar);
    void Div(const double& scalar);
};


class CVector : public CMatrix {

public:
    explicit CVector(uint32_t columns);
    CVector(uint32_t rows, uint32_t column);
    CVector(uint32_t columns, double* mat);
    CVector(uint32_t rows, uint32_t flag, double* mat);
    CVector& operator= (const CMatrix& m);
};
