function test()
%http://stackoverflow.com/questions/5059568/triangulation-using-the-direct-linear-transform-taken-directly-from-hartley-zi

  close all
  clear all

  P = [
      -1.81066 0.00000 0.00000 0.00000;
      0.00000 2.41421 0.00000 0.00000;
      0.00000 0.00000 1.00000 1.50000
      ];

  Q =  [
      -0.36118 0.00000 0.00000 0.00000
      0.00000 2.00875 1.33916 0.00000
      0.00000 -0.55470 0.83205 1.80278
      ];


  % homogenous 3D coordinates
  Pts =  [
          -0.2 -0.2 -0.2 -0.2 0.2 0.2 0.2 0.2;
          -0.2 -0.2 0.2 0.2 -0.2 -0.2 0.2 0.2;
          -0.2 0.2 -0.2 0.2 -0.2 0.2 -0.2 0.2;
          1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0
      ];    

  numPts = length(Pts(1,:));

  % project points
  PPtsHom = P*Pts;
  QPtsHom = Q*Pts;

  % normalize for homogenous scaling
  for i = 1:3
      PPtsHom(i, :) = PPtsHom(i, :) ./ PPtsHom(3, :);
      QPtsHom(i, :) = QPtsHom(i, :) ./ QPtsHom(3, :);
  end

  % 2D cartesian coordinates
  PPtsCart = PPtsHom(1:2,:)
  QPtsCart = QPtsHom(1:2,:);



  % compute normalizing transform

  % calc centroid
  centroidPPtsCart = mean(PPtsCart,2);
  centroidQPtsCart = mean(QPtsCart,2);

  % calc mean distance to centroid
  normsPPtsCart = zeros(1, numPts);
  normsQPtsCart = zeros(1, numPts);
  for i = 1:numPts
    normsPPtsCart(1,i) = norm(PPtsCart(:,i) - centroidPPtsCart);
    normsQPtsCart(1,i) = norm(QPtsCart(:,i) - centroidQPtsCart);
  end
  mdcPPtsCart = mean(normsPPtsCart);
  mdcQPtsCart = mean(normsQPtsCart);

  % setup transformation
  scaleP = sqrt(2)/mdcPPtsCart;
  scaleQ = sqrt(2)/mdcQPtsCart;

  tP = [ scaleP 0      -scaleP*centroidPPtsCart(1);
    0      scaleP -scaleP*centroidPPtsCart(2);
    0      0      1];
  tQ = [ scaleQ 0      -scaleQ*centroidQPtsCart(1);
    0      scaleQ -scaleQ*centroidQPtsCart(2);
    0      0      1];


  % transform points
  PPtsHom = tP * PPtsHom
  QPtsHom = tQ * QPtsHom;

  % normalize for homogenous scaling
  for i = 1:3
      PPtsHom(i, :) = PPtsHom(i, :) ./ PPtsHom(3, :);
      QPtsHom(i, :) = QPtsHom(i, :) ./ QPtsHom(3, :);
  end
  % 2D cartesian coordinates
  PPtsCart = PPtsHom(1:2,:);
  QPtsCart = QPtsHom(1:2,:);

  % transform cameras
  P = tP * P;
  Q = tQ * Q;


  % triangulating points
  TriangulatedPoints = zeros(4,numPts);
  for i = 1:numPts
      A = [
          PPtsCart(1, i) * P(3, :) - P(1, :);
          PPtsCart(2, i) * P(3, :) - P(2, :);
          QPtsCart(1, i) * Q(3,:) - Q(1,:);
          QPtsCart(2, i) * Q(3,:) - Q(2,:)
      ];
      %return;

      [U S V] = svd(A);
      TriangulatedPoints(:, i) = V(:, end);
  end
  for i = 1:4
      TriangulatedPoints(i, :) = TriangulatedPoints(i, :) ./ TriangulatedPoints(4, :);
  end
  TriangulatedPoints
  figure()
  scatter3(TriangulatedPoints(1, :), TriangulatedPoints(2, :), TriangulatedPoints(3, :));
  title('Triangulated Points');
  axis equal;
  