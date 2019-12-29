import numpy as np


def power_iteration(A, num_simulations: int):
    # Ideally choose a random vector
    # To decrease the chance that our vector
    # Is orthogonal to the eigenvector
    b_k = np.random.rand(A.shape[1])
    print(b_k)

    for _ in range(num_simulations):
        # calculate the matrix-by-vector product Ab
        b_k1 = np.dot(A, b_k)

        # calculate the norm
        b_k1_norm = np.linalg.norm(b_k1)
        # print('my norm: ' + str(np.sqrt(np.dot(b_k1, b_k1))))

        # re normalize the vector
        b_k = b_k1 / b_k1_norm

    return np.dot(np.dot(A, b_k), b_k) / np.dot(b_k, b_k)


def main():
    print(power_iteration(np.array([[2, -12], [1, -5]]), 10))


if __name__ == '__main__':
    main()
