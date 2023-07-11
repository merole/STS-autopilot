void mulMat(float res[3], float mat1[3][3], float mat2[3][1]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
                res[i] += mat1[i][j] * mat2[j][0];
        }
    }
}

void subMat(float res[3][1], float mat1[3][1], float mat2[3][1]) {
    for (int i = 0; i < 3; i++) {
        res[i][0] = mat1[i][0] - mat2[i][0];
    }
}
