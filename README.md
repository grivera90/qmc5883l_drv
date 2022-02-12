# qmc5883l_drv 
Se implementa un driver para controlar una brújula digital, más concretamente el qmc5883l.
Solo consta de dos funciones públicas: 

 - int qmc5883l_init(qmc5883l_t *qmc5883l, bool default_config);

 - int qmc5883l_read(qmc5883l_t *qmc5883l);
