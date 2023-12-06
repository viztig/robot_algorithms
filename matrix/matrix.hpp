#include <iostream>
#include <vector>
#pragma once
class Matrix
{
private:
    int row;
    int col;
    std::vector<std::vector<double>> matrix;

public:
    Matrix(const int &, const int &);
    Matrix(std::initializer_list<double>);
    ~Matrix();
    void init(std::initializer_list<double>);
    int rows() const { return row; }
    int cols() const { return col; }
    void set(int row, int col, int value);
    void setIdentity();
    double operator()(int, int) const;
    Matrix operator+(Matrix &) const;
    Matrix operator-(Matrix &) const;
    Matrix operator/(double &d) const;
    Matrix dot(Matrix &, Matrix &) const;
    Matrix matmul(Matrix &) const;
    Matrix transpose() const;
    Matrix inverse();
    double det() const;
    void print() const;
    double cofactor(int, int, int) const;
};
Matrix::Matrix(std::initializer_list<double> values)
{
    if (this->rows() * this->cols() != values.size())
    {
        throw std::invalid_argument("no of elements does not match matrix size");
    }
    auto it = values.begin();
    for (int i = 0; i < this->rows(); i++)
    {
        for (int j = 0; j < this->cols(); j++)
        {
            this->matrix[i][j] = *it;
            ++it;
        }
    }
}
Matrix::Matrix(const int &row, const int &col) : row(row), col(col), matrix(row, std::vector<double>(col, 0)) {}
Matrix::~Matrix()
{
}
void Matrix::init(std::initializer_list<double> values)
{
    if (this->rows() * this->cols() != values.size())
    {
        throw std::invalid_argument("no of elements does not match matrix size");
    }
    auto it = values.begin();
    for (int i = 0; i < this->rows(); i++)
    {
        for (int j = 0; j < this->cols(); j++)
        {
            this->matrix[i][j] = *it;
            ++it;
        }
    }
}
void Matrix::set(int row, int col, int value)
{
    this->matrix[row][col] = value;
}
void Matrix::setIdentity()
{
    for (int i = 0; i < this->row; i++)
    {
        for (int j = 0; j < this->col; j++)
        {
            if (i == j)
            {
                this->matrix[i][j] = 1;
            }
        }
    }
}
double Matrix::operator()(int i, int j) const
{
    if (i >= 0 && i < this->row && j >= 0 && j < this->col)
    {
        return this->matrix[i][j];
    }
    else
    {
        throw std::out_of_range("Matrix indices out of bounds");
    }
}
Matrix Matrix::operator+(Matrix &mat) const
{
    if (mat.rows() != this->rows() || mat.cols() != this->cols())
    {
        throw std::invalid_argument("Matrix sizes not compatible for addition");
    }
    Matrix res(this->rows(), this->cols());
    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            res.matrix[i][j] = (*this)(i, j) + mat(i, j);
        }
    }
    return res;
}
Matrix Matrix::operator-(Matrix &mat) const
{
    if (mat.rows() != this->rows() || mat.cols() != this->cols())
    {
        throw std::invalid_argument("Matrix sizes not compatible for substraction");
    }
    Matrix res(this->rows(), this->cols());
    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            res.matrix[i][j] = (*this)(i, j) - mat(i, j);
        }
    }
    return res;
}
Matrix Matrix::operator/(double &d) const
{
    if (d == 0)
    {
        throw std::invalid_argument("can not divide by zero");
    }
    Matrix res(this->rows(), this->cols());
    for (int i = 0; i < this->rows(); i++)
    {
        for (int j = 0; j < this->cols(); j++)
        {
            res.matrix[i][j] = (*this)(i, j) / d;
        }
    }
    return res;
}
void Matrix::print() const
{
    for (int i = 0; i < this->rows(); i++)
    {
        for (int j = 0; j < this->cols(); j++)
        {
            std::cout << (*this)(i, j) << " ";
        }
        std::cout << std::endl;
    }
}
Matrix Matrix::dot(Matrix &a, Matrix &b) const
{
    if (a.rows() != b.rows() || a.cols() != b.cols())
    {
        throw std::invalid_argument("Matrix sizes not compatible for addition");
    }
    Matrix res(a.rows(), b.cols());
    for (int i = 0; i < res.rows(); i++)
    {
        for (int j = 0; j < res.cols(); j++)
        {
            res.matrix[i][j] = a(i, j) * b(i, j);
        }
        return res;
    }
}
Matrix Matrix::matmul(Matrix &b) const
{
    if (this->cols() != b.rows())
    {
        throw std::invalid_argument("Matrix sizes not compatible for addition");
    }
    Matrix res(this->rows(), b.cols());
    for (int i = 0; i < this->rows(); i++)
    {
        for (int j = 0; j < b.cols(); j++)
        {
            for (int k = 0; k < this->cols(); k++)
            {
                res.matrix[i][j] += (*this)(i, k) * b(k, j);
            }
        }
    }
    return res;
}
double Matrix::det() const
{
    if (this->rows() != this->cols())
    {
        throw std::invalid_argument("Determinant of only square matrices can be calculated");
    }
    if (this->rows() == 1)
    {
        return (*this)(0, 0);
    }
    if (this->rows() == 2)
    {
        return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
    }
    double det = 0;
    for (int i = 0; i < this->rows(); i++)
    {
        det = det + (*this)(0, i) * this->cofactor(0, i, this->rows());
    }
    return det;
}
double Matrix::cofactor(int row, int col, int size) const
{
    Matrix cofac(size - 1, size - 1);
    for (int i = 1; i < size; i++)
    {
        int s = 0;
        for (int j = 0; j < size; j++)
            if (j != col)
            {
                cofac.matrix[i - 1][s++] = (*this)(i, j);
            }
    }
    int sign = (row + col) % 2 == 0 ? 1 : -1;
    return sign * cofac.det();
}
Matrix Matrix::transpose() const
{
    Matrix tr(this->cols(), this->rows());
    for (int i = 0; i < this->rows(); i++)
    {
        for (int j = 0; j < this->cols(); j++)
        {
            tr.matrix[i][j] = (*this)(j, i);
        }
    }
    return tr;
}
Matrix Matrix::inverse()
{
    int size = this->rows();
    if (size != this->cols())
    {
        throw std::invalid_argument("Inverse of only square matrices can be calculated");
    }
    Matrix inv(size, size);
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            inv.matrix[i][j] = (*this).cofactor(j, i, size);
        }
    }
    int det = (*this).det();
    if (det == 0)
    {
        throw std::invalid_argument("this matrix is not invertible");
    }
    return inv;
}
