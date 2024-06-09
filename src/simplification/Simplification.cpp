#include "Simplification.h"
#include "common/edge.h"
#include "common/array.h"
#include "Eigen/Core"
#include "Eigen/LU"
#include <set>
#include <map>
#include <queue>

struct costWithPoints {
    double cost;
    Point v1;
    Point v2;
    Point v;
    Eigen::Matrix4d Q;
    costWithPoints(double cost, Point v1, Point v2, Point v, Eigen::Matrix4d Q) {
        this->cost = cost;
        if (v1 < v2) {
            this->v1 = v1;
            this->v2 = v2;
        }
        else {
            this->v1 = v2;
            this->v2 = v1;
        }
        this->v = v;
        this->Q = Q;
    }

    //比較演算子をオーバーロード
    bool operator==(const costWithPoints rhs) {
        return this->cost == rhs.cost;
    }
    const bool operator==(const costWithPoints rhs) const {
        return this->cost == rhs.cost;
    }

    bool operator!=(const costWithPoints rhs) {
        return this->cost != rhs.cost;
    }
    const bool operator!=(const costWithPoints rhs) const {
        return this->cost != rhs.cost;
    }

    bool operator<(const costWithPoints rhs) {
        return this->cost < rhs.cost;
    }
    const bool operator<(const costWithPoints rhs) const {
        return this->cost < rhs.cost;
    }

    bool operator<=(const costWithPoints rhs) {
        return this->cost <= rhs.cost;
    }
    const bool operator<=(const costWithPoints rhs) const {
        return this->cost <= rhs.cost;
    }

    bool operator>(const costWithPoints rhs) {
        return this->cost > rhs.cost;
    }
    const bool operator>(const costWithPoints rhs) const {
        return this->cost > rhs.cost;
    }

    bool operator>=(const costWithPoints rhs) {
        return this->cost >= rhs.cost;
    }
    const bool operator>=(const costWithPoints rhs) const {
        return this->cost >= rhs.cost;
    }

    //デバッグ用の情報表示
    inline void displayD() {
        printf("座標(v1): %lf, %lf, %lf 座標(v2): %lf, %lf, %lf\n", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
        printf("座標(newV): %lf, %lf, %lf\n\n", v.x, v.y, v.z);
    }
};

//被っている三角形を排除する
void deleteSameTriangles(std::vector<Triangle>& generateTriangles) {
    std::set<Triangle> generateTriangles_set;
    for (int i = 0; i < generateTriangles.size(); i++) {
        generateTriangles_set.insert(generateTriangles[i]);
    }
    generateTriangles.clear();
    for (Triangle tri : generateTriangles_set) {
        generateTriangles.push_back(tri);
    }
}

void simplifyQEM(std::vector<Point>& points, std::vector<Triangle>& generateTriangles, const int faceSize) {
    std::set<Triangle> generateTriangles_set;
    deleteSameTriangles(generateTriangles);

    //generateTrianglesからpoint2triを構築してpointからQを返すmapを作る
        //そのためにpointからtrianlgeへの配列を用意する
        //ついでにvalidPairにおいて辺の条件を満たすものを構築する
        //さらにコスト計算時に不連続であるペアにペナルティを課すために、ここで境界辺を見つけておく
        //生成する三角形の配列のsetバージョンを作っておく
    std::vector<std::vector<Triangle>> point2tri(points.size(), std::vector<Triangle>(0));
    std::set<Edge> validPair;
    std::set<Edge> boundaryEdge;
    std::vector<std::vector<Edge>> point2edge(points.size(), std::vector<Edge>(0));
    std::map<Edge, Triangle> edge2tri;

    for (int i = 0; i < generateTriangles.size(); i++) {
        Triangle tri = generateTriangles[i];
        point2tri[tri.vertex1.index].push_back(tri);
        point2tri[tri.vertex2.index].push_back(tri);
        point2tri[tri.vertex3.index].push_back(tri);

        //setの作成
        generateTriangles_set.insert(tri);

        //辺の格納
        Edge e1 = Edge(tri.vertex1, tri.vertex2);
        Edge e2 = Edge(tri.vertex2, tri.vertex3);
        Edge e3 = Edge(tri.vertex3, tri.vertex1);
        validPair.insert(e1); validPair.insert(e2); validPair.insert(e3);

        point2edge[tri.vertex1.index].push_back(e3); point2edge[tri.vertex1.index].push_back(e1);
        point2edge[tri.vertex2.index].push_back(e1); point2edge[tri.vertex2.index].push_back(e2);
        point2edge[tri.vertex3.index].push_back(e2); point2edge[tri.vertex3.index].push_back(e3);

        edge2tri.insert(std::make_pair(e1, tri)); edge2tri.insert(std::make_pair(e2, tri)); edge2tri.insert(std::make_pair(e3, tri));

        //境界辺の格納
        if (boundaryEdge.find(e1) == boundaryEdge.end()) boundaryEdge.insert(e1);
        else boundaryEdge.erase(e1);
        if (boundaryEdge.find(e2) == boundaryEdge.end()) boundaryEdge.insert(e2);
        else boundaryEdge.erase(e2);
        if (boundaryEdge.find(e3) == boundaryEdge.end()) boundaryEdge.insert(e3);
        else boundaryEdge.erase(e3);
    }

    //不連続になっている辺に辺を通る垂直平面を追加して境界上の点にペナルティを課す
    for (auto e : boundaryEdge) {
        Point p1 = e.vertex1;
        Point p2 = e.vertex2;
        Point virtualP = p1;
        Array tri_normal = edge2tri.at(e).calcNormal();
        virtualP.x += tri_normal.x; virtualP.y += tri_normal.y; virtualP.z += tri_normal.z;
        Triangle tri = Triangle(p1, p2, virtualP);
        tri.virtualTri = true;
        point2tri[p1.index].push_back(tri);
        point2tri[p2.index].push_back(tri);
    }

    //各点でQを作ってpoint2Qに入れる
    std::map<Point, Eigen::Matrix4d> point2Q;
    for (int i = 0; i < points.size(); i++) {
        Eigen::Matrix4d Q = Eigen::Matrix4d::Zero(4, 4);
        //QはKjの和で表される
        for (int j = 0; j < point2tri[i].size(); j++) {
            Eigen::Matrix4d Kj;
            Triangle tri = point2tri[i][j];
            std::vector<double> cofficients = tri.calcCofficients();
            //Kjに係数の積を入れていく
            for (int m = 0; m < 4; m++) {
                for (int n = 0; n < 4; n++) {
                    Kj(m, n) = cofficients[m] * cofficients[n];
                }
            }
            //Qに加算
            Q += Kj;
        }
        point2Q.insert(std::make_pair(points[i], Q));
    }

    printf("valid pair構築完了\n");
    std::priority_queue<costWithPoints, std::vector<costWithPoints>, std::greater<costWithPoints>> pq;
    //優先度付きキューにコストと3つの点を入れていく
    for (Edge e : validPair) {
        //コストを求めるためのQ_を計算し、v_を求めるためのhomogeneousQ_も計算する
        Eigen::Matrix4d Q_ = point2Q.at(e.vertex1) + point2Q.at(e.vertex2);
        Eigen::Matrix4d homogeneousQ_ = Q_;
        homogeneousQ_(3, 0) = 0.0; homogeneousQ_(3, 1) = 0.0; homogeneousQ_(3, 2) = 0.0; homogeneousQ_(3, 3) = 1.0;

        //v_をhomogeneousQから求める
        Eigen::Vector4d b;
        b(0) = 0.0; b(1) = 0.0; b(2) = 0.0; b(3) = 1.0;
        Eigen::Vector4d v_ = homogeneousQ_.partialPivLu().solve(b);

        //コストを求めて優先度付きキューに入れる
        double cost = v_.transpose() * Q_ * v_;
        //v_からPointとしてのv_を求める
        Point optimazedV = Point(v_(0), v_(1), v_(2));
        pq.emplace(costWithPoints(cost, e.vertex1, e.vertex2, optimazedV, Q_));
    }

    int pN = points.size();
    int progressVal = -1;
    const int needDeleteNum1000 = (generateTriangles_set.size() - faceSize) / 1000;
    const int defaultSize = generateTriangles_set.size();
    std::set<Point> deletedPoint;
    //更新処理
    while (generateTriangles_set.size() > faceSize) {
        //
        int nowVal = (defaultSize - generateTriangles_set.size()) / needDeleteNum1000;
        if (nowVal > progressVal) {
            printf("\r\033[0K");
            printf("simplification %d/1000", progressVal);
            progressVal = nowVal;
        }

        //最もコストが小さいものを取り出し、処理を行う
        costWithPoints cwp = pq.top();
        pq.pop();
        Point v1 = cwp.v1; Point v2 = cwp.v2;
        //既に削除済みのものが絡んだペアであれば飛ばす。そうでなければ削除する
        if (deletedPoint.find(v1) != deletedPoint.end() || deletedPoint.find(v2) != deletedPoint.end()) {
            continue;
        }
        deletedPoint.insert(v1); deletedPoint.insert(v2);

        //新たな点の追加
        Point newP = cwp.v;
        newP.index = pN;
        pN++;
        newP.normal = v1.normal + v2.normal;
        newP.normal.normalize();
        newP.r = (v1.r + v2.r) / 2; newP.g = (v1.g + v2.g) / 2; newP.b = (v1.b + v2.b) / 2;
        newP.CTval = (v1.CTval + v2.CTval) / 2;
        points.push_back(newP);
        point2Q.insert(std::make_pair(newP, cwp.Q));

        //生成する三角形群に三角形を追加・削除する
        //ついでにnewP側のpoint2edgeとpoint2triも更新する
        std::set<Triangle> updateTri;
        std::set<Point> adjacentV1V2;
        std::vector<Triangle> newP2tri;
        //v1に隣接する三角形とその点を追加していく
        for (int i = 0; i < point2tri[v1.index].size(); i++) {
            Triangle tri = point2tri[v1.index][i];
            updateTri.insert(tri);
            adjacentV1V2.insert(tri.vertex1);
            adjacentV1V2.insert(tri.vertex2);
            adjacentV1V2.insert(tri.vertex3);
        }
        //v2も同様に追加していく
        for (int i = 0; i < point2tri[v2.index].size(); i++) {
            Triangle tri = point2tri[v2.index][i];
            adjacentV1V2.insert(tri.vertex1);
            adjacentV1V2.insert(tri.vertex2);
            adjacentV1V2.insert(tri.vertex3);
            //v1とv2で追加した三角形が同じ時は削除する
            if (updateTri.find(tri) != updateTri.end()) {
                updateTri.erase(tri);
                generateTriangles_set.erase(tri);
            }
            else {
                updateTri.insert(tri);
            }
        }

        //v1orv2のみ持つ三角形のv1をnewPに変更して追加
        for (Triangle tri : updateTri) {
            generateTriangles_set.erase(tri);
            //v1,v2をnewPに変換した結果三角形ではなくなったらスキップ
            if (!tri.convertPoints(v1, v2, newP)) continue;
            //境界辺にペナルティとして追加した三角形だったらスキップ
            if (!tri.virtualTri) generateTriangles_set.insert(tri);

            newP2tri.push_back(tri);
        }

        //追加する点に隣接する辺と三角形を求める
        std::vector<Edge> newP2edge;
        for (auto p : adjacentV1V2) {
            if (p == v1 || p == v2) continue;
            newP2edge.push_back(Edge(newP, p));
        }
        point2tri.push_back(newP2tri);
        point2edge.push_back(newP2edge);

        //v1,v2に隣接する点のpoint2edgeとpoint2triを更新する
        for (Point adjacentP : adjacentV1V2) {
            if (adjacentP == v1 || adjacentP == v2) continue;
            //point2edgeの更新
            for (int i = 0; i < point2edge[adjacentP.index].size(); i++) {
                point2edge[adjacentP.index][i].convertPoint(v1, newP);
                point2edge[adjacentP.index][i].convertPoint(v2, newP);
            }
            //point2triの更新
            std::vector<Triangle> changedPointTri;
            for (int i = 0; i < point2tri[adjacentP.index].size(); i++) {
                //もし点の変更が1点以下であれば更新し、配列に入れる
                if (point2tri[adjacentP.index][i].convertPoints(v1, v2, newP)) {
                    changedPointTri.push_back(point2tri[adjacentP.index][i]);
                }
            }
            point2tri[adjacentP.index] = changedPointTri;
        }

        //validPair,優先度付きキューの更新
            //新たなvalidPairを探索
        std::vector<Edge> updateEdge;
        for (int i = 0; i < point2edge[v1.index].size(); i++) {
            if (point2edge[v1.index][i].vertex1 == v1) {
                updateEdge.push_back(Edge(newP, point2edge[v1.index][i].vertex2));
            }
            else {
                updateEdge.push_back(Edge(newP, point2edge[v1.index][i].vertex1));
            }
        }
        for (int i = 0; i < point2edge[v2.index].size(); i++) {
            if (point2edge[v2.index][i].vertex1 == v2) {
                updateEdge.push_back(Edge(newP, point2edge[v2.index][i].vertex2));
            }
            else {
                updateEdge.push_back(Edge(newP, point2edge[v2.index][i].vertex1));
            }
        }

        //新たなvalidPairのコストを計算し、優先度付きキューに入れる
        for (int i = 0; i < updateEdge.size(); i++) {
            //コストを求めるためのQ_を計算し、v_を求めるためのhomogeneousQ_も計算する
            Eigen::Matrix4d Q_ = point2Q.at(updateEdge[i].vertex1) + point2Q.at(updateEdge[i].vertex2);
            Eigen::Matrix4d homogeneousQ_ = Q_;
            homogeneousQ_(3, 0) = 0.0; homogeneousQ_(3, 1) = 0.0; homogeneousQ_(3, 2) = 0.0; homogeneousQ_(3, 3) = 1.0;

            //v_をhomogeneousQから求める
            Eigen::Vector4d b;
            b(0) = 0.0; b(1) = 0.0; b(2) = 0.0; b(3) = 1.0;
            Eigen::Vector4d v_ = homogeneousQ_.partialPivLu().solve(b);

            //コストを求めて優先度付きキューに入れる
            double cost = v_.transpose() * Q_ * v_;
            //v_からPointとしてのv_を求める
            Point optimazedV = Point(v_(0), v_(1), v_(2));
            pq.emplace(costWithPoints(cost, updateEdge[i].vertex1, updateEdge[i].vertex2, optimazedV, Q_));
        }
    }
    printf("\n");
    printf("simplification終了\n");
    generateTriangles.clear();
    for (Triangle tri : generateTriangles_set) {
        //デバッグ
        Point v1 = tri.vertex1; Point v2 = tri.vertex2; Point v3 = tri.vertex3;
        if (v1 == v2 || v2 == v3 || v3 == v1) {
            printf("error\n");
            continue;
        }

        generateTriangles.push_back(tri);
    }
}