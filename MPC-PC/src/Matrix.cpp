#include "Matrix.h"
#include <cstring>


CMatrix::CMatrix(uint32_t rows, uint32_t columns) : rows(rows), columns(columns) {
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = 0;
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns, double** mat) : rows(rows), columns(columns) {
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i][j];
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns, const double* mat) : rows(rows), columns(columns) {
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i * columns + j];
        }
    }
}


CMatrix::CMatrix(const CMatrix& M) : rows(M.rows), columns(M.columns) {
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = M.matrix[i][j];
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns, const std::initializer_list<double>& list) : rows(rows), columns(columns) {
    auto iter = list.begin();
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = *iter++;
        }
    }
}


CMatrix::CMatrix(uint32_t rows, const std::string& value) : rows(rows), columns(rows) {
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            if (!std::strcmp(value.c_str(), "eye")) {
                if (i == j) {
                    matrix[i][j] = 1;
                } else {
                    matrix[i][j] = 0;
                }
            } else {
                matrix[i][j] = std::atof(value.c_str());
            }
        }
    }
}


CMatrix::~CMatrix() {
    for (uint32_t i = 0; i < rows; i++) {
        delete[] matrix[i];
    }
    delete[] matrix;
}


void CMatrix::GetSize(uint32_t& rows, uint32_t& columns) const {
    rows = this->rows;
    columns = this->columns;
}

double& CMatrix::GetElement(uint32_t row, uint32_t column) const {
    if (row < 0 || column < 0 || row >= rows || column >= columns) {
        throw std::invalid_argument("GetElement: Requested index is out of range");
    }
    return matrix[row][column];
}


double* CMatrix::operator[](uint32_t row) const {
    if (row < 0 || row >= rows) {
        throw std::invalid_argument("[]: Requested index is out of range");
    }
    return matrix[row];
}


void CMatrix::SetRow(uint32_t row, const CMatrix& v) {
    if (v.rows != 1) {
        throw std::invalid_argument("SetRow: Number of source's rows is greater than 1. CVector is required");
    }
    else if (v.columns != columns) {
        throw std::invalid_argument("SetRow: Mismatch of source and destination size");
    }
    else if (row < 0 || row >= rows) {
        throw std::invalid_argument("SetRow: Requested index is out of range");
    }
    for (uint32_t j = 0; j < columns; j++) {
        matrix[row][j] = v[0][j];
    }
}


void CMatrix::SetColumn(uint32_t column, const CMatrix& v) {
    if (v.columns != 1) {
        throw std::invalid_argument("SetColumn: Number of source's columns is greater than 1. CVector is required");
    }
    else if (v.rows != rows) {
        throw std::invalid_argument("SetColumn: Mismatch of source and destination size");
    }
    if (column < 0 || column >= columns) {
        throw std::invalid_argument("SetColumn: Requested index is out of range");
    }
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i][column] = v[i][0];
    }
}


double CMatrix::Det() const {
    if (rows != columns) {
        throw std::domain_error("Det: Square matrix is needed to obtain its' determinant");
    }
    if (rows == 1) {
        return matrix[0][0];
    }
    double det = 0;
    CMatrix cofactor(rows - 1, columns - 1);
    for (uint32_t j = 0; j < columns; j++) {
        cofactor = GetCofactor(0, j);
        if (j % 2) {
            det -= matrix[0][j] * Det(std::move(cofactor), rows - 1);
        } else {
            det += matrix[0][j] * Det(std::move(cofactor), rows - 1);
        }
    }
    return det;
}


CMatrix& CMatrix:: operator= (const CMatrix& m) {
    if (rows != m.rows || columns != m.columns) {
        throw std::invalid_argument("Assign: Mismatch of matrices' dimensions");
    }
    if (this == &m) {
        return *this;
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = m.matrix[i][j];
        }
    }
    return *this;
}


CMatrix CMatrix::operator+ (const CMatrix& m) const {
    CMatrix mat(*this);
    mat.Add(m);
    return mat;
}


CMatrix& CMatrix::operator+= (const CMatrix& m) {
    this->Add(m);
    return *this;
}


CMatrix CMatrix::operator- (const CMatrix& m) const {
    CMatrix mat(*this);
    mat.Sub(m);
    return mat;
}


CMatrix& CMatrix::operator-= (const CMatrix& m) {
    this->Sub(m);
    return *this;
}


CMatrix CMatrix::operator* (const CMatrix& m) const {
    return this->Mul(m);
}


CMatrix& CMatrix::operator*= (const CMatrix& m) {
    *this = this->Mul(m);
    return *this;
}


CMatrix CMatrix::operator* (const double& scalar) const {
    CMatrix mat(*this);
    mat.Mul(scalar);
    return mat;
}


CMatrix& CMatrix::operator*= (const double& scalar) {
    this->Mul(scalar);
    return *this;
}


CMatrix CMatrix::operator/ (const double& scalar) const {
    CMatrix mat(*this);
    mat.Div(scalar);
    return mat;
}


CMatrix& CMatrix::operator/= (const double& scalar) {
    this->Div(scalar);
    return *this;
}


CMatrix CMatrix::operator-() const {
    CMatrix mat(*this);
    mat.Mul(-1);
    return mat;
}


CMatrix CMatrix::operator^ (const uint32_t& exponent) const {
    if (rows != columns) {
        throw std::invalid_argument("^: Matrix has to be square");
    }
    CMatrix mat(rows, "eye");
    for (uint32_t i = 0; i < exponent; i++) {
        mat *= *this;
    }
    return mat;
}


CMatrix CMatrix::T() const {
    CMatrix matrix_t(columns, rows);
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix_t.matrix[j][i] = matrix[i][j];
        }
    }
    return matrix_t;
}


CMatrix CMatrix::operator()() {
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = 0;
        }
    }
    return *this;
}


CMatrix CMatrix::operator()(uint32_t rows, uint32_t columns) {
    this->~CMatrix();
    this->rows = rows;
    this->columns = columns;
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = 0;
        }
    }
    return *this;
}


CMatrix CMatrix::operator()(uint32_t rows, uint32_t columns, const double* mat) {
    this->~CMatrix();
    this->rows = rows;
    this->columns = columns;
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i * columns + j];
        }
    }
    return *this;
}


CMatrix CMatrix::operator()(uint32_t rows, uint32_t columns, const std::initializer_list<double>& list) {
    this->~CMatrix();
    this->rows = rows;
    this->columns = columns;
    auto iter = list.begin();
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = *iter++;
        }
    }
    return *this;
}


std::ostream& operator<< (std::ostream& stream, const CMatrix& m) {
    uint32_t rows, columns;
    m.GetSize(rows, columns);
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            stream << m.matrix[i][j] << " ";
        }
        stream << std::endl;
    }
    return stream;
}


void CMatrix::make_matrix() {
    matrix = new double*[rows];
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
}


void CMatrix::Add(const CMatrix& m) {
    if (rows != m.rows || columns != m.columns) {
        throw std::invalid_argument("Add: Mismatch of matrices' dimensions");
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] += m.matrix[i][j];
        }
    }
}


void CMatrix::Sub(const CMatrix& m) {
    if (rows != m.rows || columns != m.columns) {
        throw std::invalid_argument("Sub: Mismatch of matrices' dimensions");
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] -= m.matrix[i][j];
        }
    }
}


CMatrix CMatrix::Mul(const CMatrix& m) const {
    if (columns != m.rows) {
        throw std::invalid_argument("Mul: Mismatch of matrices' dimensions");
    }
    CMatrix productMatrix(rows, m.columns);
    for (uint32_t i = 0; i < productMatrix.rows; i++) {
        for (uint32_t j = 0; j < productMatrix.columns; j++) {
            for (uint32_t k = 0; k < columns; k++) {
                productMatrix.matrix[i][j] += (matrix[i][k] * m.matrix[k][j]);
            }
        }
    }
    return productMatrix;
}


void CMatrix::Mul(const double& scalar) {
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] *= scalar;
        }
    }
}


void CMatrix::Div(const double& scalar) {
    if (scalar == 0) {
        throw std::invalid_argument("Div: Cannot divide by 0");
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] /= scalar;
        }
    }
}


CMatrix CMatrix::GetCofactor(uint32_t row, uint32_t column) const {
    CMatrix cofactor(rows - 1, columns - 1);
    int c_i = 0, c_j = 0;
    for (uint32_t i = 0; i < rows; i++) {
        c_j = 0;
        for (uint32_t j = 0; j < columns; j++) {
            if (i != row && j != column) {
                cofactor[c_i][c_j++] = matrix[i][j];
            }
        }
        if (i != row) {
            c_i++;
        }
    }
    return cofactor;
}


double CMatrix::Det(CMatrix&& cofactor, uint32_t iteration) const {
    if (cofactor.rows != cofactor.columns) {
        throw std::domain_error("Det: Square matrix is needed to obtain its' determinant");
    }
    if (iteration == 1) {
        return cofactor[0][0];
    }
    double det = 0;
    for (uint32_t j = 0; j < iteration; j++) {
        if (j % 2) {
            det -= cofactor[0][j] * Det(cofactor.GetCofactor(0, j), iteration - 1);
        } else {
            det += cofactor[0][j] * Det(cofactor.GetCofactor(0, j), iteration - 1);
        }
    }
    return det;
}


CVector::CVector(uint32_t columns) : CMatrix(1, columns) {}


CVector::CVector(uint32_t rows, uint32_t column) : CMatrix(rows, 1) {}


CVector::CVector(uint32_t columns, double* mat) : CMatrix(1, columns, mat) {}


CVector::CVector(uint32_t rows, uint32_t column, double* mat) : CMatrix(rows, 1, mat) {}


CVector::CVector(uint32_t columns, const std::string& value) : CMatrix(1, columns) {
    make_matrix();
    for (uint32_t j = 0; j < columns; j++) {
        matrix[0][j] = std::atof(value.c_str());
    }
}


CVector::CVector(uint32_t rows, uint32_t column, const std::string& value) : CMatrix(rows, 1) {
    make_matrix();
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i][0] = std::atof(value.c_str());
    }
}


CVector& CVector::operator= (const CMatrix& m) {
    uint32_t m_rows, m_columns;
    m.GetSize(m_rows, m_columns);
    if (m_rows != rows || m_columns != columns) {
        throw std::invalid_argument("Assign: Mismatch of vector's dimensions");
    }
    if (m_rows == 1) {
        for (uint32_t i = 0; i < m_columns; i++) {
            matrix[0][i] = m.GetElement(0, i);
        }
    }
    else {
        for (uint32_t i = 0; i < m_rows; i++) {
            matrix[i][0] = m.GetElement(i, 0);
        }
    }
    return *this;
}
