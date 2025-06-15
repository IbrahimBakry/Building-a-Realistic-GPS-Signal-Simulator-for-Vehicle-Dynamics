function gps_simulator_full()
    % Параметры автомобиля
    L = 2;                  % Расстояние между осями (м)
    a = L/2;                % Расстояние от ЦТ до передней оси
    b = L/2;                % Расстояние от ЦТ до задней оси
    m = 1500;               % Масса автомобиля (кг)
    Iz = 3000;              % Момент инерции (кг*м^2)
    Cfx = 80000;           % Продольная жесткость шин (Н)
    Cfy = 60000;            % Боковая жесткость шин (Н/рад)
    Cd = 0.3;               % Коэффициент аэродинамического сопротивления
    rc = 0.3;               % Эффективный радиус колеса (м)
    
    % Параметры эллипсоида WGS84
    a_earth = 6378137.0;        % Большая полуось (м)
    f_earth = 1/298.257223563;  % Сжатие
    e2_earth = (2*f_earth - f_earth^2); % Квадрат эксцентриситета
    omega_earth = 7.2921159e-5; % Угловая скорость Земли (рад/с)
    
    % Начальные координаты
    lat0 = 53.262778;       % Начальная широта (град)
    lon0 = 50.372778;       % Начальная долгота (град)
    h0 = 0;                 % Высота (м)
    lat0_rad = deg2rad(lat0);
    lon0_rad = deg2rad(lon0);
    
    % Преобразование начальных координат в ECEF
    [X0, Y0, Z0] = geodetic2ecef(lat0_rad, lon0_rad, h0, a_earth, f_earth);
    
    % Матрица преобразования ENU->ECEF
    R_enu2ecef = [-sin(lon0_rad), -sin(lat0_rad)*cos(lon0_rad), cos(lat0_rad)*cos(lon0_rad);
                   cos(lon0_rad), -sin(lat0_rad)*sin(lon0_rad), cos(lat0_rad)*sin(lon0_rad);
                   0,             cos(lat0_rad),                sin(lat0_rad)];
    
    % Ввод параметров пользователем
    speed_kmh = input('Введите скорость (0-30 км/ч): ');
    steering_angle = input('Введите угол поворота руля (-600..+600 град): ');
    method = input('Выберите метод интегрирования ("1" Euler, "2" ode45): ');
    
    % Преобразование параметров
    v_desired = speed_kmh * 1000 / 3600;   % Желаемая скорость (м/с)
    delta_deg = steering_angle / 20;       % Преобразование угла
    delta = deg2rad(delta_deg);            % Угол поворота в радианах
    
    % Инициализация переменных состояния
    state = [v_desired;    % U - продольная скорость
             0;            % V - боковая скорость
             0;            % Omega_z - угловая скорость
             0;            % Theta - угол курса
             0;            % X East (локальная ENU)
             0];           % Y North (локальная ENU)
    
    dt = 0.1;              % Шаг интегрирования (с)
    sim_time = 0;          % Время симуляции
    
    % Создание графического окна
    fig = figure('Name', 'Vehicle Trajectory', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);
    ax = axes('Position', [0.1, 0.1, 0.85, 0.85]);
    hold on;
    grid on;
    title('Траектория движения автомобиля');
    xlabel('Долгота (°)');
    ylabel('Широта (°)');
    
    % Начальная точка
    plot(lon0, lat0, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Начальная точка');
    trajectory = plot(lon0, lat0, 'b-', 'DisplayName', 'Траектория');
    current_pos = plot(lon0, lat0, 'ro', 'MarkerSize', 8, 'DisplayName', 'Текущая позиция');
    legend('Location', 'best');
    
    % Основной цикл
    while ishandle(fig)
        % 1. Извлечение текущего состояния
        U = state(1); V = state(2); Omega_z = state(3); 
        Theta = state(4); X_East = state(5); Y_North = state(6);
        
        % 2. Расчет скольжения шин
        if abs(U) > 0.5
            w_front = U / rc;
            s1 = (w_front * rc - U) / U;
            s1 = max(min(s1, 0.3), -0.3);
        else
            s1 = 0;
        end
        s2 = s1;
        s3 = 0;
        s4 = 0;
        
        % 3. Расчет углов alpha
        if abs(U) > 0.5
            alpha1 = delta - (V + a*Omega_z) / U;
            alpha3 = - (V - b*Omega_z) / U;

            %Ограничение углов увода (±20 градусов)
            max_alpha = deg2rad(20);
            alpha1 = max(min(alpha1, max_alpha), -max_alpha);
            alpha3 = max(min(alpha3, max_alpha), -max_alpha);
        else
            alpha1 = 0;
            alpha3 = 0;
        end
        
        % 4. Расчет сил
        Fx1 = Cfx * s1; Fx2 = Cfx * s2; Fx3 = Cfx * s3; Fx4 = Cfx * s4;
        Fy1 = Cfy * alpha1; Fy2 = Cfy * alpha1; Fy3 = Cfy * alpha3; Fy4 = Cfy * alpha3;

        % Ограничение сил (±8000 Н)
        max_force = 6000;
        Fx1 = max(min(Fx1, max_force), -max_force);
        Fx2 = max(min(Fx2, max_force), -max_force);
        Fy1 = max(min(Fy1, max_force), -max_force);
        Fy2 = max(min(Fy2, max_force), -max_force);
        Fy3 = max(min(Fy3, max_force), -max_force);
        Fy4 = max(min(Fy4, max_force), -max_force);

        % 5. Расчет моментов
        Fx = (Fx1 + Fx2)*cos(delta) + (Fx3 + Fx4) - (Fy1 + Fy2)*sin(delta) - Cd*U^2;
        Fy = (Fx1 + Fx2)*sin(delta) + (Fy1 + Fy2)*cos(delta) + (Fy3 + Fy4);
        Mz = a*((Fx1 + Fx2)*sin(delta) + (Fy1 + Fy2)*cos(delta)) - b*(Fy3 + Fy4);
        
        % 6. Дифференциальные уравнения в СВ СК
        dU_dt = Fx/m + V*Omega_z;
        dV_dt = Fy/m - U*Omega_z;
        dOmega_z_dt = Mz/Iz;
        dTheta_dt = Omega_z;
        dEast_dt = U*sin(Theta) + V*cos(Theta);
        dNorth_dt = U*cos(Theta) - V*sin(Theta);
        
        % 7. Интегрирование
        if method == 1 % Euler
            state(1) = U + dU_dt * dt;
            state(2) = V + dV_dt * dt;
            state(3) = Omega_z + dOmega_z_dt * dt;
            state(4) = Theta + dTheta_dt * dt;
            state(5) = X_East + dEast_dt * dt;
            state(6) = Y_North + dNorth_dt * dt;
        else % ode45
            f = @(t,y) [dU_dt; dV_dt; dOmega_z_dt; dTheta_dt; dEast_dt; dNorth_dt];
            [~, y] = ode45(f, [0 dt], state);
            state = y(end, :)';
        end
        
        % Стабилизация модели
        % Ограничение скоростей
        max_U_speed = 30; % m/s
        max_V_speed = 10; % m/s
        state(1) = max(min(state(1), max_U_speed), -max_U_speed);
        state(2) = max(min(state(2), max_V_speed), -max_V_speed); % Боковая скорость
        
        % Ограничение угловой скорости
        max_yaw_rate = 2; % rad/s
        state(3) = max(min(state(3), max_yaw_rate), -max_yaw_rate);
        
        % Нормализация угла курса
        state(4) = atan2(sin(state(4)), cos(state(4)));


        % 8. Контроль скорости
        speed_error = v_desired - state(1);
        state(1) = state(1) + 0.1 *speed_error;

        % 9. Преобразование координат
        % 9.1. Локальные ENU -> ECEF
        dECEF = R_enu2ecef * [state(5); state(6); 0];
        X = X0 + dECEF(1);
        Y = Y0 + dECEF(2);
        Z = Z0 + dECEF(3);
        
        % 9.2. ECEF -> Геодезические
        [lat_rad, lon_rad, h] = ecef2geodetic(X, Y, Z, a_earth, f_earth);
        lat = rad2deg(lat_rad);
        lon = rad2deg(lon_rad);
        
        % 10. Обновление графика
        xdata = [get(trajectory, 'XData'), lon];
        ydata = [get(trajectory, 'YData'), lat];
        set(trajectory, 'XData', xdata, 'YData', ydata);
        set(current_pos, 'XData', lon, 'YData', lat);
        
        % Автомасштабирование только при разумных значениях
        if all(isfinite(xdata)) && all(isfinite(ydata)) && ...
           abs(lon) < 180 && abs(lat) < 90
            min_lon = min(xdata);
            max_lon = max(xdata);
            min_lat = min(ydata);
            max_lat = max(ydata);
            lon_margin = max(0.0001, 0.1 * (max_lon - min_lon));
            lat_margin = max(0.0001, 0.1 * (max_lat - min_lat));
            
            axis(ax, [min_lon-lon_margin, max_lon+lon_margin, ...
                      min_lat-lat_margin, max_lat+lat_margin]);
        end
        
        drawnow;
        
        % 11. Генерация NMEA сообщений
        hours = floor(sim_time / 3600);
        mins = floor((sim_time - hours*3600) / 60);
        secs = mod(sim_time, 60);
        nmea_time = sprintf('%02d%02d%05.2f', hours, mins, secs);
        
        [lat_str, lon_str] = convert_coords(lat, lon);
        gga = format_gga(nmea_time, lat_str, lon_str, h);
        vtg = format_vtg(rad2deg(Theta), speed_kmh);
        
        disp(gga);
        disp(vtg);
        
        % 12. Обновление времени
        sim_time = sim_time + dt;
        pause(dt);
    end
end

function [X, Y, Z] = geodetic2ecef(lat, lon, h, a, f)
    % Преобразование геодезических координат в ECEF
    e2 = 2*f - f^2;
    N = a / sqrt(1 - e2 * sin(lat)^2);
    
    X = (N + h) * cos(lat) * cos(lon);
    Y = (N + h) * cos(lat) * sin(lon);
    Z = (N * (1 - e2) + h) * sin(lat);
end

function [lat, lon, h] = ecef2geodetic(X, Y, Z, a, f)
    % Преобразование ECEF в геодезические координаты
    e2 = 2*f - f^2;
    lon = atan2(Y, X);
    
    p = sqrt(X^2 + Y^2);
    lat0 = atan2(Z, p * (1 - e2));
    tol = 1e-11;
    max_iter = 100;
    
    for i = 1:max_iter
        N = a / sqrt(1 - e2 * sin(lat0)^2);
        h = p / cos(lat0) - N;
        lat = atan2(Z, p * (1 - e2 * N/(N + h)));
        
        if abs(lat - lat0) < tol
            break;
        end
        lat0 = lat;
    end
end

function [lat_str, lon_str] = convert_coords(lat, lon)
    % Форматирование координат для NMEA
    lat_deg = floor(abs(lat));
    lat_min = (abs(lat) - lat_deg) * 60;
    lat_str = sprintf('%02d%09.6f', lat_deg, lat_min);
    
    lon_deg = floor(abs(lon));
    lon_min = (abs(lon) - lon_deg) * 60;
    lon_str = sprintf('%03d%09.6f', lon_deg, lon_min);
end

function gga = format_gga(time, lat, lon, h)
    % Формирование GGA сообщения
    template = ['$GPGGA,' ...
        '%s,' ...    % Время
        '%s,' ...    % Широта
        'N,' ...     % Направление широты
        '%s,' ...    % Долгота
        'E,' ...     % Направление долготы
        ',' ...      % Качество фиксации
        ',' ...      % Количество спутников
        ',' ...      % HDOP
        '%.1f,' ...  % Высота
        'M,' ...     % Единицы высоты
        '0.0,M,,,' ... % Геоидальная высота
        '*%s'];      % Контрольная сумма
    
    data_str = sprintf('GPGGA,%s,%s,N,%s,E,,,,%.1f,M,0.0,M,,', ...
        time, lat, lon, h);
    checksum = nmea_checksum(data_str);
    gga = sprintf(template, time, lat, lon, h, checksum);
end

function vtg = format_vtg(course, speed_kmh)
    % Формирование VTG сообщения
    speed_knots = speed_kmh / 1.852; % Конвертация в узлы
    
    template = ['$GPVTG,' ...
        '%.1f,T,' ...    % Истинный курс
        ',,' ...         % Магнитный курс
        ',%.1f,N,' ...   % Скорость в узлах
        ',%.1f,K,' ...   % Скорость в км/ч
        'A*%s'];         % Режим и контрольная сумма
    
    data_str = sprintf('GPVTG,%.1f,T,,,%.1f,N,%.1f,K,A', ...
        course, speed_knots, speed_kmh);
    checksum = nmea_checksum(data_str);
    vtg = sprintf(template, course, speed_knots, speed_kmh, checksum);
end

function checksum = nmea_checksum(str)
    % Расчет контрольной суммы NMEA
    cs = uint8(0);
    for i = 1:length(str)
        cs = bitxor(cs, uint8(str(i)));
    end
    checksum = dec2hex(cs, 2);
end