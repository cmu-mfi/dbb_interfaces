/****** Object:  StoredProcedure [dbo].[ProjectList]    Script Date: 11/7/2024 3:33:21 PM ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author: SSR
-- Create date: <Create Date,,>
-- Description:	Gets ProjectId, Title, Description, Organization, and Members from Projects table
-- =============================================
CREATE PROCEDURE [dbo].[ProjectList] 

AS
BEGIN
	-- SET NOCOUNT ON added to prevent extra result sets from
	-- interfering with SELECT statements.
	SET NOCOUNT ON;

    -- Insert statements for procedure here
	select	ProjectID 
			,Title
			, Description
			, Organization
			, Members
	from dbo.Projects with (nolock)
END
